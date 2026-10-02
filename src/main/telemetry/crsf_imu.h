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

/*
 * CRSF IMU telemetry: integrated gyro (delta angle) and accelerometer
 * (delta velocity) increments, batched into one extended-header CRSF frame.
 * The wire format is documented in IMU_FRAME_SPEC.md, distributed with this patch.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/streambuf.h"
#include "common/time.h"

#include "pg/pg.h"

typedef enum {
    CRSF_IMU_ACCEL_OFF = 0,     // gyro only
    CRSF_IMU_ACCEL_ON,          // delta velocity for every interval
    CRSF_IMU_ACCEL_FRAME,       // one delta velocity over the whole frame
} crsfImuAccelMode_e;

typedef struct crsfImuConfig_s {
    uint8_t enabled;            // OFF / ON
    uint8_t accelMode;          // crsfImuAccelMode_e
    uint8_t batch;              // intervals per frame (clamped to what fits the frame)
    uint8_t frameType;          // CRSF frame type used for the IMU frame
    uint16_t intervalUs;        // integration interval
    uint8_t stdTlmDiv;          // divider for the regular CRSF telemetry while IMU telemetry is on
} crsfImuConfig_t;

PG_DECLARE(crsfImuConfig_t, crsfImuConfig);

#define CRSF_IMU_FORMAT_VERSION         1
#define CRSF_IMU_DEFAULT_FRAME_TYPE     0x7F

#define CRSF_IMU_INTERVAL_US_MIN        1000
#define CRSF_IMU_INTERVAL_US_MAX        20000
#define CRSF_IMU_BATCH_MAX              8
#define CRSF_IMU_STD_TLM_DIV_MAX        20

// data bytes after the extended header (destination, origin)
#define CRSF_IMU_HEADER_SIZE            10
#define CRSF_IMU_DATA_SIZE_MAX          58  // CRSF_FRAME_SIZE_MAX - sync, len, type, dest, orig, crc
#define CRSF_IMU_VECTOR_SIZE            6   // 3 x int16

// maximum number of intervals per frame for each accelerometer mode
#define CRSF_IMU_BATCH_MAX_ACCEL_OFF    8   // 10 + 8 * 6  = 58
#define CRSF_IMU_BATCH_MAX_ACCEL_ON     4   // 10 + 4 * 12 = 58
#define CRSF_IMU_BATCH_MAX_ACCEL_FRAME  7   // 10 + 7 * 6 + 6 = 58

// fixed point scales (value = raw * scale)
#define CRSF_IMU_GYRO_SCALE_RAD         1e-4f   // delta angle, rad per LSB
#define CRSF_IMU_ACCEL_SCALE_MPS        1e-4f   // delta velocity per interval, m/s per LSB
#define CRSF_IMU_ACCEL_FRAME_SCALE_MPS  1e-3f   // delta velocity per frame, m/s per LSB
#define CRSF_IMU_RAW_MAX                32767

// flags byte
#define CRSF_IMU_FLAG_ARMED             (1 << 0)
#define CRSF_IMU_FLAG_ANGLE_MODE        (1 << 1)
#define CRSF_IMU_FLAG_GYRO_NOT_READY    (1 << 2)    // gyro calibration in progress, data invalid
#define CRSF_IMU_FLAG_SATURATED         (1 << 3)    // at least one value in this frame was clipped
#define CRSF_IMU_FLAG_FC_DROPPED        (1 << 4)    // FC discarded at least one frame since the previous sent one
#define CRSF_IMU_FLAG_ACCEL_SHIFT       5           // bits 5..6: crsfImuAccelMode_e
#define CRSF_IMU_FLAG_ACCEL_MASK        (3 << CRSF_IMU_FLAG_ACCEL_SHIFT)

typedef struct crsfImuFrameData_s {
    uint16_t seq;
    uint32_t endTimeUs;
    uint16_t intervalUs;
    uint8_t flags;              // SATURATED, FC_DROPPED and accel mode bits
    uint8_t count;              // number of intervals
    int16_t gyro[CRSF_IMU_BATCH_MAX][3];
    int16_t accel[CRSF_IMU_BATCH_MAX][3];   // ACCEL_ON: one per interval, ACCEL_FRAME: accel[0] only
} crsfImuFrameData_t;

// true when IMU telemetry is configured and running; checked by the PID loop before accumulating
extern bool crsfImuActive;

void crsfImuInit(uint32_t pidLooptimeUs, bool accAvailable, uint16_t acc1G);
void crsfImuAccumulate(const float gyroDps[3], const float accRaw[3], timeUs_t currentTimeUs);

bool crsfImuIsActive(void);
bool crsfImuPopFrame(crsfImuFrameData_t *frame);
uint8_t crsfImuStateFlags(void);
uint8_t crsfImuStdTlmDiv(void);
uint8_t crsfImuFrameType(void);
uint16_t crsfImuIntervalUs(void);
uint8_t crsfImuBatchSize(void);

// writes length, type, extended header and data; the caller adds sync byte and CRC
void crsfImuSerializeFrame(sbuf_t *dst, const crsfImuFrameData_t *frame, uint8_t frameType, uint8_t stateFlags);
