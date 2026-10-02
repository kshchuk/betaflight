/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_CRSF

#include "common/axis.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/crsf_protocol.h"

#include "sensors/gyro.h"

#include "telemetry/crsf_imu.h"

#define CRSF_IMU_GRAVITY_MSS 9.80665f

PG_REGISTER_WITH_RESET_TEMPLATE(crsfImuConfig_t, crsfImuConfig, PG_CRSF_IMU_CONFIG, 0);

PG_RESET_TEMPLATE(crsfImuConfig_t, crsfImuConfig,
    .enabled = 0,
    .accelMode = CRSF_IMU_ACCEL_OFF,
    .batch = CRSF_IMU_BATCH_MAX,
    .frameType = CRSF_IMU_DEFAULT_FRAME_TYPE,
    .intervalUs = 5000,
    .stdTlmDiv = 5,
);

FAST_DATA_ZERO_INIT bool crsfImuActive;

// state touched by the PID loop
static FAST_DATA_ZERO_INIT struct {
    float gyroSum[XYZ_AXIS_COUNT];      // in output LSB, carries the rounding residual
    float accelSum[XYZ_AXIS_COUNT];     // in output LSB, carries the rounding residual
    float gyroGain;                     // deg/s -> LSB per PID loop
    float accelGain;                    // acc LSB -> output LSB per PID loop
    uint16_t loopsPerInterval;
    uint16_t loopCount;
    uint8_t sampleCount;
    uint8_t batchSize;
    uint8_t accelMode;
    uint8_t satFlags;
} imu;

static crsfImuFrameData_t workFrame;
static crsfImuFrameData_t readyFrame;
static bool readyValid;
static bool dropPending;
static uint16_t nextSeq;
static uint16_t intervalUs;
static uint8_t frameType;
static uint8_t stdTlmDiv = 1;

static uint8_t batchMaxForAccelMode(uint8_t accelMode)
{
    switch (accelMode) {
    case CRSF_IMU_ACCEL_ON:
        return CRSF_IMU_BATCH_MAX_ACCEL_ON;
    case CRSF_IMU_ACCEL_FRAME:
        return CRSF_IMU_BATCH_MAX_ACCEL_FRAME;
    default:
        return CRSF_IMU_BATCH_MAX_ACCEL_OFF;
    }
}

void crsfImuInit(uint32_t pidLooptimeUs, bool accAvailable, uint16_t acc1G)
{
    crsfImuActive = false;
    memset(&imu, 0, sizeof(imu));
    memset(&workFrame, 0, sizeof(workFrame));
    memset(&readyFrame, 0, sizeof(readyFrame));
    readyValid = false;
    dropPending = false;
    nextSeq = 0;
    stdTlmDiv = 1;

    const crsfImuConfig_t *config = crsfImuConfig();
    if (!config->enabled || pidLooptimeUs == 0) {
        return;
    }

    uint8_t accelMode = config->accelMode;
    if (!accAvailable || acc1G == 0 || accelMode > CRSF_IMU_ACCEL_FRAME) {
        accelMode = CRSF_IMU_ACCEL_OFF;
    }
    imu.accelMode = accelMode;
    imu.batchSize = constrain(config->batch, 1, batchMaxForAccelMode(accelMode));

    // the interval is a whole number of PID loops, so every interval has exactly the same length
    const uint32_t requestedUs = constrain(config->intervalUs, CRSF_IMU_INTERVAL_US_MIN, CRSF_IMU_INTERVAL_US_MAX);
    uint32_t loops = (requestedUs + pidLooptimeUs / 2) / pidLooptimeUs;
    loops = constrain(loops, 1, UINT16_MAX / pidLooptimeUs);
    imu.loopsPerInterval = loops;
    intervalUs = loops * pidLooptimeUs;

    const float dtS = pidLooptimeUs * 1e-6f;
    imu.gyroGain = dtS * RAD / CRSF_IMU_GYRO_SCALE_RAD;
    if (accelMode != CRSF_IMU_ACCEL_OFF) {
        const float accelScale = (accelMode == CRSF_IMU_ACCEL_FRAME) ? CRSF_IMU_ACCEL_FRAME_SCALE_MPS : CRSF_IMU_ACCEL_SCALE_MPS;
        imu.accelGain = dtS * CRSF_IMU_GRAVITY_MSS / acc1G / accelScale;
    }

    frameType = config->frameType;
    stdTlmDiv = constrain(config->stdTlmDiv, 1, CRSF_IMU_STD_TLM_DIV_MAX);

    crsfImuActive = true;
}

// Round to the nearest LSB and keep the residual in the sum, so the reported
// increments never drift from the true integral. Clipped values drop the excess.
static int16_t crsfImuQuantize(float *sum, uint8_t *satFlags)
{
    const float value = *sum;
    if (value >= (float)CRSF_IMU_RAW_MAX) {
        *sum = 0.0f;
        *satFlags |= CRSF_IMU_FLAG_SATURATED;
        return CRSF_IMU_RAW_MAX;
    }
    if (!(value > -(float)CRSF_IMU_RAW_MAX)) { // also catches NaN
        *sum = 0.0f;
        *satFlags |= CRSF_IMU_FLAG_SATURATED;
        return -CRSF_IMU_RAW_MAX;
    }
    const int32_t rounded = (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
    *sum = value - (float)rounded;
    return (int16_t)rounded;
}

static void crsfImuCompleteInterval(timeUs_t currentTimeUs)
{
    const uint8_t index = imu.sampleCount;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        workFrame.gyro[index][axis] = crsfImuQuantize(&imu.gyroSum[axis], &imu.satFlags);
    }
    if (imu.accelMode == CRSF_IMU_ACCEL_ON) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            workFrame.accel[index][axis] = crsfImuQuantize(&imu.accelSum[axis], &imu.satFlags);
        }
    }

    imu.sampleCount++;
    if (imu.sampleCount < imu.batchSize) {
        return;
    }

    if (imu.accelMode == CRSF_IMU_ACCEL_FRAME) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            workFrame.accel[0][axis] = crsfImuQuantize(&imu.accelSum[axis], &imu.satFlags);
        }
    }

    if (readyValid) {
        // previous frame was never sent: it is replaced, newest data wins
        dropPending = true;
    }

    workFrame.seq = nextSeq++;
    workFrame.endTimeUs = currentTimeUs;
    workFrame.intervalUs = intervalUs;
    workFrame.count = imu.sampleCount;
    workFrame.flags = imu.satFlags
        | (dropPending ? CRSF_IMU_FLAG_FC_DROPPED : 0)
        | (imu.accelMode << CRSF_IMU_FLAG_ACCEL_SHIFT);

    readyFrame = workFrame;
    readyValid = true;

    imu.sampleCount = 0;
    imu.satFlags = 0;
}

// Called from the PID loop after the motor update. Only multiply-adds and a
// counter, the interval/frame bookkeeping runs once per interval.
FAST_CODE void crsfImuAccumulate(const float gyroDps[3], const float accRaw[3], timeUs_t currentTimeUs)
{
    const float gyroGain = imu.gyroGain;
    imu.gyroSum[X] += gyroDps[X] * gyroGain;
    imu.gyroSum[Y] += gyroDps[Y] * gyroGain;
    imu.gyroSum[Z] += gyroDps[Z] * gyroGain;

    if (imu.accelMode != CRSF_IMU_ACCEL_OFF) {
        const float accelGain = imu.accelGain;
        imu.accelSum[X] += accRaw[X] * accelGain;
        imu.accelSum[Y] += accRaw[Y] * accelGain;
        imu.accelSum[Z] += accRaw[Z] * accelGain;
    }

    if (++imu.loopCount < imu.loopsPerInterval) {
        return;
    }
    imu.loopCount = 0;

    crsfImuCompleteInterval(currentTimeUs);
}

bool crsfImuIsActive(void)
{
    return crsfImuActive;
}

bool crsfImuPopFrame(crsfImuFrameData_t *frame)
{
    if (!readyValid) {
        return false;
    }
    *frame = readyFrame;
    readyValid = false;
    dropPending = false;
    return true;
}

uint8_t crsfImuStateFlags(void)
{
    uint8_t flags = 0;
    if (ARMING_FLAG(ARMED)) {
        flags |= CRSF_IMU_FLAG_ARMED;
    }
    if (FLIGHT_MODE(ANGLE_MODE)) {
        flags |= CRSF_IMU_FLAG_ANGLE_MODE;
    }
    if (!gyroIsCalibrationComplete()) {
        flags |= CRSF_IMU_FLAG_GYRO_NOT_READY;
    }
    return flags;
}

uint8_t crsfImuStdTlmDiv(void)
{
    return crsfImuActive ? stdTlmDiv : 1;
}

uint8_t crsfImuFrameType(void)
{
    return frameType;
}

uint16_t crsfImuIntervalUs(void)
{
    return intervalUs;
}

uint8_t crsfImuBatchSize(void)
{
    return imu.batchSize;
}

void crsfImuSerializeFrame(sbuf_t *dst, const crsfImuFrameData_t *frame, uint8_t type, uint8_t stateFlags)
{
    const uint8_t accelMode = (frame->flags & CRSF_IMU_FLAG_ACCEL_MASK) >> CRSF_IMU_FLAG_ACCEL_SHIFT;
    const uint8_t count = MIN(frame->count, batchMaxForAccelMode(accelMode));

    uint8_t dataSize = CRSF_IMU_HEADER_SIZE + count * CRSF_IMU_VECTOR_SIZE;
    if (accelMode == CRSF_IMU_ACCEL_ON) {
        dataSize += count * CRSF_IMU_VECTOR_SIZE;
    } else if (accelMode == CRSF_IMU_ACCEL_FRAME) {
        dataSize += CRSF_IMU_VECTOR_SIZE;
    }

    sbufWriteU8(dst, dataSize + CRSF_FRAME_LENGTH_EXT_TYPE_CRC);
    sbufWriteU8(dst, type);
    sbufWriteU8(dst, CRSF_ADDRESS_RADIO_TRANSMITTER);
    sbufWriteU8(dst, CRSF_ADDRESS_FLIGHT_CONTROLLER);

    sbufWriteU8(dst, CRSF_IMU_FORMAT_VERSION);
    sbufWriteU8(dst, frame->flags | stateFlags);
    sbufWriteU16BigEndian(dst, frame->seq);
    sbufWriteU32BigEndian(dst, frame->endTimeUs);
    sbufWriteU16BigEndian(dst, frame->intervalUs);

    for (int i = 0; i < count; i++) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            sbufWriteU16BigEndian(dst, (uint16_t)frame->gyro[i][axis]);
        }
        if (accelMode == CRSF_IMU_ACCEL_ON) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                sbufWriteU16BigEndian(dst, (uint16_t)frame->accel[i][axis]);
            }
        }
    }
    if (accelMode == CRSF_IMU_ACCEL_FRAME) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            sbufWriteU16BigEndian(dst, (uint16_t)frame->accel[0][axis]);
        }
    }
}

#endif // USE_TELEMETRY_CRSF
