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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmath>
#include <vector>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/crc.h"
    #include "common/maths.h"
    #include "common/streambuf.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/serial.h"
    #include "drivers/system.h"

    #include "config/config.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"
    #include "rx/crsf.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "telemetry/crsf.h"
    #include "telemetry/crsf_imu.h"
    #include "telemetry/telemetry.h"
    #include "telemetry/msp_shared.h"

    #include "drivers/persistent.h"
    #include "msp/msp.h"

    rssiSource_e rssiSource;

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------
// test harness: a fake CRSF serial port that records every frame written to it

struct CapturedFrame {
    uint32_t timeUs;
    std::vector<uint8_t> bytes;
    uint8_t type() const { return bytes[2]; }
};

static std::vector<CapturedFrame> capturedFrames;
static uint32_t testTimeUs = 1000000;   // monotonic across all tests, the telemetry code keeps static timestamps
static bool gyroCalibrated = true;

static const float accOneG = 2048.0f;   // ICM42688P acc_1G

static void advanceTo(uint32_t timeUs)
{
    testTimeUs = timeUs;
}

static void startCrsfTelemetry(void)
{
    static rxRuntimeState_t rxRuntimeState;
    crsfRxInit(rxConfig(), &rxRuntimeState);
    sensorsSet(SENSOR_ACC);
    acc.dev.acc_1G = (uint16_t)accOneG;
    targetPidLooptime = 250;    // 8 kHz gyro, pid_process_denom 2
    initCrsfTelemetry();
    capturedFrames.clear();
}

static void configureImu(bool enabled, uint16_t intervalUs, uint8_t batch, uint8_t accelMode, uint8_t stdTlmDiv)
{
    crsfImuConfigMutable()->enabled = enabled;
    crsfImuConfigMutable()->intervalUs = intervalUs;
    crsfImuConfigMutable()->batch = batch;
    crsfImuConfigMutable()->accelMode = accelMode;
    crsfImuConfigMutable()->frameType = CRSF_IMU_DEFAULT_FRAME_TYPE;
    crsfImuConfigMutable()->stdTlmDiv = stdTlmDiv;
}

// Runs the PID loop (every 250us) and the telemetry task (every 2ms) for durationUs.
static void runLoops(uint32_t durationUs, const float gyroDps[3], const float accRaw[3])
{
    const uint32_t end = testTimeUs + durationUs;
    while (testTimeUs < end) {
        testTimeUs += 250;
        if (crsfImuActive) {
            crsfImuAccumulate(gyroDps, accRaw, testTimeUs);
        }
        if (testTimeUs % 2000 == 0) {
            handleCrsfTelemetry(testTimeUs);
        }
    }
    handleCrsfTelemetry(testTimeUs); // flush whatever was queued last
}

static uint8_t referenceCrc8D5(const uint8_t *data, int len)
{
    // bitwise CRC-8 poly 0xD5, independent of common/crc.c
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void expectValidCrsfFrame(const std::vector<uint8_t> &frame)
{
    ASSERT_GE(frame.size(), 4u);
    EXPECT_LE(frame.size(), (size_t)CRSF_FRAME_SIZE_MAX);
    EXPECT_EQ(CRSF_SYNC_BYTE, frame[0]);
    EXPECT_EQ(frame.size(), (size_t)frame[1] + 2);
    EXPECT_EQ(referenceCrc8D5(&frame[2], frame.size() - 3), frame.back());
}

static int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static uint16_t beU16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t beU32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

static std::vector<uint8_t> serializeWithSyncAndCrc(const crsfImuFrameData_t *data, uint8_t frameType, uint8_t stateFlags)
{
    uint8_t buf[CRSF_FRAME_SIZE_MAX + 8];
    sbuf_t sbuf;
    sbuf_t *dst = sbufInit(&sbuf, buf, buf + sizeof(buf));
    sbufWriteU8(dst, CRSF_SYNC_BYTE);
    crsfImuSerializeFrame(dst, data, frameType, stateFlags);
    crc8_dvb_s2_sbuf_append(dst, &buf[2]);
    return std::vector<uint8_t>(buf, dst->ptr);
}

static std::vector<uint8_t> hexToBytes(const char *hex)
{
    std::vector<uint8_t> out;
    while (*hex) {
        if (*hex == ' ') {
            hex++;
            continue;
        }
        char byteStr[3] = { hex[0], hex[1], 0 };
        out.push_back((uint8_t)strtoul(byteStr, NULL, 16));
        hex += 2;
    }
    return out;
}

static void dumpFrame(const char *name, const std::vector<uint8_t> &frame)
{
    printf("%s len=%u", name, (unsigned)frame.size());
    for (uint8_t b : frame) {
        printf(" %02X", b);
    }
    printf("\n");
}

static int countType(uint8_t type)
{
    int n = 0;
    for (const CapturedFrame &f : capturedFrames) {
        n += (f.type() == type);
    }
    return n;
}

// ---------------------------------------------------------------------------
// 1. Telemetry OFF: no IMU frame, regular schedule identical to stock 4.5.0.
//    Must run first, so the static schedule index in telemetry/crsf.c starts at 0.

TEST(TelemetryCrsfImuTest, OffKeepsStockSchedule)
{
    configureImu(false, 5000, 8, CRSF_IMU_ACCEL_OFF, 5);
    startCrsfTelemetry();
    EXPECT_FALSE(crsfImuActive);
    EXPECT_EQ(1, crsfImuStdTlmDiv());

    const float gyroDps[3] = { 100.0f, 200.0f, 300.0f };
    const float accRaw[3] = { 0.0f, 0.0f, accOneG };
    advanceTo(2000000);
    runLoops(2000000, gyroDps, accRaw);

#if defined(USE_CRSF_V3)
    // stock schedule with CRSF V3: attitude, battery, flight mode, GPS and a heartbeat, one slot every 20ms
    const uint8_t expectedOrder[] = { 0x1E, 0x08, 0x21, 0x02, 0x0B };
    const uint32_t expectedSpacingUs = 20000;
#else
    // stock schedule for this test target: attitude, battery, flight mode, GPS round robin,
    // one frame per 100ms / 4 slots, i.e. the first 2ms tick after 25ms -> every 26ms
    const uint8_t expectedOrder[] = { 0x1E, 0x08, 0x21, 0x02 };
    const uint32_t expectedSpacingUs = 26000;
#endif
    const size_t slots = ARRAYLEN(expectedOrder);
    ASSERT_GT(capturedFrames.size(), 70u);
    for (size_t i = 0; i < capturedFrames.size(); i++) {
        EXPECT_EQ(expectedOrder[i % slots], capturedFrames[i].type()) << "frame " << i;
        expectValidCrsfFrame(capturedFrames[i].bytes);
        if (i > 0) {
            EXPECT_EQ(expectedSpacingUs, capturedFrames[i].timeUs - capturedFrames[i - 1].timeUs) << "frame " << i;
        }
    }
    EXPECT_EQ(0, countType(CRSF_IMU_DEFAULT_FRAME_TYPE));

    if (getenv("CRSF_SCHEDULE_DUMP")) {
        // used to diff this schedule against the same harness built from unmodified 4.5.0 sources
        FILE *f = fopen(getenv("CRSF_SCHEDULE_DUMP"), "w");
        for (const CapturedFrame &frame : capturedFrames) {
            fprintf(f, "%u", frame.timeUs);
            for (uint8_t b : frame.bytes) {
                fprintf(f, " %02X", b);
            }
            fprintf(f, "\n");
        }
        fclose(f);
    }

    // the PID loop hook is a no-op when OFF
    capturedFrames.clear();
    runLoops(100000, gyroDps, accRaw);
    crsfImuFrameData_t frame;
    EXPECT_FALSE(crsfImuPopFrame(&frame));
}

// ---------------------------------------------------------------------------
// 2. Frame encoding: known values -> exact bytes (vectors from the independent Python encoder)

struct EncodingVector {
    uint8_t frameType;
    uint8_t stateFlags;     // ARMED / ANGLE / GYRO_NOT_READY, added at send time
    uint8_t frameFlags;     // SATURATED / FC_DROPPED, set by the accumulator
    uint8_t accelMode;
    uint16_t seq;
    uint32_t endTimeUs;
    uint16_t intervalUs;
    uint8_t count;
    int16_t gyro[8][3];
    int16_t accel[8][3];
    const char *hex;
};

static const EncodingVector encodingVectors[] = {
    { 0x7F, CRSF_IMU_FLAG_ARMED, 0, CRSF_IMU_ACCEL_OFF, 41, 12345678, 5000, 8,
      { { 87, -44, 436 }, { 88, -43, 437 }, { 87, -44, 436 }, { 87, -44, 437 },
        { 88, -44, 436 }, { 87, -43, 436 }, { 87, -44, 437 }, { 88, -44, 436 } }, { },
      "C8 3E 7F EA C8 01 01 00 29 00 BC 61 4E 13 88 00 57 FF D4 01 B4 00 58 FF D5 01 B5 00 57 FF D4 01 B4 00 57 FF D4 01 B5 00 58 FF D4 01 B4 00 57 FF D5 01 B4 00 57 FF D4 01 B5 00 58 FF D4 01 B4 7D" },
    { 0x7F, CRSF_IMU_FLAG_ARMED | CRSF_IMU_FLAG_ANGLE_MODE, 0, CRSF_IMU_ACCEL_ON, 65535, 4000000000u, 2500, 4,
      { { 0, 0, 0 }, { 1, 0, -1 }, { 0, -1, 0 }, { -1, 1, 1 } },
      { { 3, -2, 245 }, { 2, -1, 245 }, { 3, -2, 246 }, { 2, -2, 245 } },
      "C8 3E 7F EA C8 01 23 FF FF EE 6B 28 00 09 C4 00 00 00 00 00 00 00 03 FF FE 00 F5 00 01 00 00 FF FF 00 02 FF FF 00 F5 00 00 FF FF 00 00 00 03 FF FE 00 F6 FF FF 00 01 00 01 00 02 FF FE 00 F5 6D" },
    { 0x7F, 0, 0, CRSF_IMU_ACCEL_FRAME, 1000, 60000000, 7500, 7,
      { { 0, 262, 0 }, { 0, 262, 0 }, { 0, 261, 0 }, { 0, 262, 0 }, { 0, 262, 0 }, { 0, 262, 0 }, { 0, 261, 0 } },
      { { -12, 0, 515 } },
      "C8 3E 7F EA C8 01 40 03 E8 03 93 87 00 1D 4C 00 00 01 06 00 00 00 00 01 06 00 00 00 00 01 05 00 00 00 00 01 06 00 00 00 00 01 06 00 00 00 00 01 06 00 00 00 00 01 05 00 00 FF F4 00 00 02 03 BF" },
    { 0x7F, CRSF_IMU_FLAG_GYRO_NOT_READY, CRSF_IMU_FLAG_SATURATED | CRSF_IMU_FLAG_FC_DROPPED, CRSF_IMU_ACCEL_OFF, 7, 1500000, 20000, 2,
      { { 32767, -32767, 0 }, { -32767, 32767, 12345 } }, { },
      "C8 1A 7F EA C8 01 1C 00 07 00 16 E3 60 4E 20 7F FF 80 01 00 00 80 01 7F FF 30 39 1C" },
    { 0x80, CRSF_IMU_FLAG_ARMED, 0, CRSF_IMU_ACCEL_OFF, 0, 0xFFFFFFF0u, 1000, 1,
      { { -1, -2, -3 } }, { },
      "C8 14 80 EA C8 01 01 00 00 FF FF FF F0 03 E8 FF FF FF FE FF FD F8" },
};

TEST(TelemetryCrsfImuTest, EncodingMatchesReferenceVectors)
{
    int index = 1;
    for (const EncodingVector &v : encodingVectors) {
        crsfImuFrameData_t data;
        memset(&data, 0, sizeof(data));
        data.seq = v.seq;
        data.endTimeUs = v.endTimeUs;
        data.intervalUs = v.intervalUs;
        data.count = v.count;
        data.flags = v.frameFlags | (v.accelMode << CRSF_IMU_FLAG_ACCEL_SHIFT);
        memcpy(data.gyro, v.gyro, sizeof(data.gyro));
        memcpy(data.accel, v.accel, sizeof(data.accel));

        const std::vector<uint8_t> frame = serializeWithSyncAndCrc(&data, v.frameType, v.stateFlags);
        expectValidCrsfFrame(frame);
        EXPECT_EQ(hexToBytes(v.hex), frame) << "vector " << index;

        char name[16];
        snprintf(name, sizeof(name), "V%d", index);
        dumpFrame(name, frame);
        index++;
    }
}

TEST(TelemetryCrsfImuTest, FullFramesFitIn64Bytes)
{
    const uint8_t modes[] = { CRSF_IMU_ACCEL_OFF, CRSF_IMU_ACCEL_ON, CRSF_IMU_ACCEL_FRAME };
    for (uint8_t mode : modes) {
        crsfImuFrameData_t data;
        memset(&data, 0, sizeof(data));
        data.count = CRSF_IMU_BATCH_MAX;   // more than fits: serializer must clamp
        data.flags = mode << CRSF_IMU_FLAG_ACCEL_SHIFT;
        const std::vector<uint8_t> frame = serializeWithSyncAndCrc(&data, 0x7F, 0);
        EXPECT_EQ((size_t)CRSF_FRAME_SIZE_MAX, frame.size()) << "mode " << (int)mode;
        EXPECT_EQ(62, frame[1]);
    }
}

// ---------------------------------------------------------------------------
// 3. Integration: constant rate over many loops -> exact integral, no rounding drift

static std::vector<crsfImuFrameData_t> runAccumulator(uint32_t loops, const float gyroDps[3], const float accRaw[3])
{
    std::vector<crsfImuFrameData_t> frames;
    uint32_t t = 0;
    for (uint32_t i = 0; i < loops; i++) {
        t += 250;
        crsfImuAccumulate(gyroDps, accRaw, t);
        crsfImuFrameData_t frame;
        if (crsfImuPopFrame(&frame)) {
            frames.push_back(frame);
        }
    }
    return frames;
}

TEST(TelemetryCrsfImuTest, ConstantRateIntegratesWithoutDrift)
{
    configureImu(true, 5000, 8, CRSF_IMU_ACCEL_ON, 5);
    crsfImuInit(250, true, (uint16_t)accOneG);
    ASSERT_TRUE(crsfImuActive);
    EXPECT_EQ(5000, crsfImuIntervalUs());
    EXPECT_EQ(4, crsfImuBatchSize());   // ACCEL_ON fits 4 intervals

    const float gyroDps[3] = { 123.4f, -45.6f, 777.7f };
    const float accRaw[3] = { 0.1f * accOneG, -0.25f * accOneG, 1.0f * accOneG };
    const uint32_t loops = 40000; // 10 s at 4 kHz
    const std::vector<crsfImuFrameData_t> frames = runAccumulator(loops, gyroDps, accRaw);

    ASSERT_EQ(loops / 20 / 4, frames.size());
    double sumGyro[3] = { 0, 0, 0 };
    double sumAcc[3] = { 0, 0, 0 };
    for (const crsfImuFrameData_t &frame : frames) {
        EXPECT_EQ(4, frame.count);
        EXPECT_EQ(0, frame.flags & (CRSF_IMU_FLAG_SATURATED | CRSF_IMU_FLAG_FC_DROPPED));
        for (int i = 0; i < frame.count; i++) {
            for (int axis = 0; axis < 3; axis++) {
                sumGyro[axis] += frame.gyro[i][axis];
                sumAcc[axis] += frame.accel[i][axis];
                // each interval within 1 LSB of the exact per-interval increment
                const double exactGyro = gyroDps[axis] * M_PI / 180.0 * 0.005 / 1e-4;
                EXPECT_NEAR(exactGyro, frame.gyro[i][axis], 1.0);
            }
        }
    }
    const double seconds = loops * 250e-6;
    for (int axis = 0; axis < 3; axis++) {
        const double exactAngle = gyroDps[axis] * M_PI / 180.0 * seconds / 1e-4;            // LSB
        const double exactVelocity = accRaw[axis] / accOneG * 9.80665 * seconds / 1e-4;    // LSB
        // error feedback keeps the total within one LSB (float accumulation adds a tiny bit)
        EXPECT_NEAR(exactAngle, sumGyro[axis], 1.0) << "axis " << axis;
        EXPECT_NEAR(exactVelocity, sumAcc[axis], 1.0) << "axis " << axis;
    }
}

TEST(TelemetryCrsfImuTest, AccelPerFrameMode)
{
    configureImu(true, 7500, 8, CRSF_IMU_ACCEL_FRAME, 5);
    crsfImuInit(250, true, (uint16_t)accOneG);
    EXPECT_EQ(7500, crsfImuIntervalUs());
    EXPECT_EQ(7, crsfImuBatchSize());

    const float gyroDps[3] = { 0.0f, 200.0f, 0.0f };
    const float accRaw[3] = { 0.0f, 0.0f, accOneG };
    const std::vector<crsfImuFrameData_t> frames = runAccumulator(30 * 7 * 10, gyroDps, accRaw);
    ASSERT_EQ(10u, frames.size());
    for (const crsfImuFrameData_t &frame : frames) {
        EXPECT_EQ(CRSF_IMU_ACCEL_FRAME, (frame.flags & CRSF_IMU_FLAG_ACCEL_MASK) >> CRSF_IMU_FLAG_ACCEL_SHIFT);
        // 1 g over 7 x 7.5ms = 0.5148 m/s at 1e-3 m/s per LSB
        EXPECT_NEAR(514.85, frame.accel[0][Z], 1.0);
        EXPECT_NEAR(0, frame.accel[0][X], 0.5);
        for (int i = 0; i < frame.count; i++) {
            EXPECT_NEAR(261.8, frame.gyro[i][Y], 1.0);
        }
    }
}

TEST(TelemetryCrsfImuTest, IntervalIsWholeNumberOfPidLoops)
{
    configureImu(true, 3333, 8, CRSF_IMU_ACCEL_OFF, 5);
    crsfImuInit(250, true, (uint16_t)accOneG);
    EXPECT_EQ(3250, crsfImuIntervalUs());     // 13 loops

    configureImu(true, 100, 8, CRSF_IMU_ACCEL_OFF, 5);      // below minimum: clamped
    crsfImuInit(250, true, (uint16_t)accOneG);
    EXPECT_EQ(1000, crsfImuIntervalUs());

    configureImu(true, 5000, 8, CRSF_IMU_ACCEL_ON, 5);      // no accelerometer: gyro only
    crsfImuInit(250, false, 0);
    EXPECT_EQ(8, crsfImuBatchSize());
    const float gyroDps[3] = { 1.0f, 2.0f, 3.0f };
    const std::vector<crsfImuFrameData_t> frames = runAccumulator(160, gyroDps, NULL);
    ASSERT_EQ(1u, frames.size());
    EXPECT_EQ(CRSF_IMU_ACCEL_OFF, (frames[0].flags & CRSF_IMU_FLAG_ACCEL_MASK) >> CRSF_IMU_FLAG_ACCEL_SHIFT);
}

// ---------------------------------------------------------------------------
// 4. Saturation: no wrap-around, flag set, next interval clean

TEST(TelemetryCrsfImuTest, SaturationClipsWithoutWrap)
{
    configureImu(true, 20000, 2, CRSF_IMU_ACCEL_ON, 5);
    crsfImuInit(250, true, (uint16_t)accOneG);

    const float hugeDps[3] = { 1e6f, -1e6f, 0.0f };
    const float hugeAcc[3] = { 100.0f * accOneG, -100.0f * accOneG, 0.0f };
    std::vector<crsfImuFrameData_t> frames = runAccumulator(80 * 2, hugeDps, hugeAcc);
    ASSERT_EQ(1u, frames.size());
    EXPECT_TRUE(frames[0].flags & CRSF_IMU_FLAG_SATURATED);
    for (int i = 0; i < 2; i++) {
        EXPECT_EQ(32767, frames[0].gyro[i][X]);
        EXPECT_EQ(-32767, frames[0].gyro[i][Y]);
        EXPECT_EQ(0, frames[0].gyro[i][Z]);
        EXPECT_EQ(32767, frames[0].accel[i][X]);
        EXPECT_EQ(-32767, frames[0].accel[i][Y]);
    }

    // back to a normal rate: the clipped excess must not leak into later intervals
    const float normalDps[3] = { 100.0f, -100.0f, 0.0f };
    const float normalAcc[3] = { 0.0f, 0.0f, accOneG };
    frames = runAccumulator(80 * 2, normalDps, normalAcc);
    ASSERT_EQ(1u, frames.size());
    EXPECT_FALSE(frames[0].flags & CRSF_IMU_FLAG_SATURATED);
    EXPECT_NEAR(349.07, frames[0].gyro[0][X], 1.0);    // 100 deg/s * 20ms
    EXPECT_NEAR(-349.07, frames[0].gyro[0][Y], 1.0);
    EXPECT_NEAR(1961.3, frames[0].accel[0][Z], 1.0);   // 1 g * 20ms

    const float nanDps[3] = { NAN, 0.0f, 0.0f };
    frames = runAccumulator(80 * 2, nanDps, normalAcc);
    ASSERT_EQ(1u, frames.size());
    EXPECT_TRUE(frames[0].flags & CRSF_IMU_FLAG_SATURATED);
}

// ---------------------------------------------------------------------------
// 5. Sequence numbers, timestamps and FC-side drops

TEST(TelemetryCrsfImuTest, SequenceTimestampAndDrops)
{
    configureImu(true, 5000, 4, CRSF_IMU_ACCEL_OFF, 5);
    crsfImuInit(250, true, (uint16_t)accOneG);

    const float gyroDps[3] = { 10.0f, 20.0f, 30.0f };
    uint32_t t = 0xFFFF0000u;   // close to the micros() wrap
    std::vector<crsfImuFrameData_t> frames;
    for (int i = 0; i < 80 * 6; i++) {
        t += 250;
        crsfImuAccumulate(gyroDps, NULL, t);
        crsfImuFrameData_t frame;
        if (crsfImuPopFrame(&frame)) {
            frames.push_back(frame);
        }
    }
    ASSERT_EQ(6u, frames.size());
    for (size_t i = 0; i < frames.size(); i++) {
        EXPECT_EQ(i, frames[i].seq);
        EXPECT_EQ(5000, frames[i].intervalUs);
        EXPECT_EQ(0, frames[i].flags & CRSF_IMU_FLAG_FC_DROPPED);
        // end of the last interval = time of the PID loop that completed it
        EXPECT_EQ((uint32_t)(0xFFFF0000u + (i + 1) * 20000), frames[i].endTimeUs);
    }

    // nobody pops for three frames: two are replaced, the newest one carries the drop flag
    for (int i = 0; i < 80 * 3; i++) {
        t += 250;
        crsfImuAccumulate(gyroDps, NULL, t);
    }
    crsfImuFrameData_t frame;
    ASSERT_TRUE(crsfImuPopFrame(&frame));
    EXPECT_EQ(8, frame.seq);    // 6 and 7 never left the FC
    EXPECT_TRUE(frame.flags & CRSF_IMU_FLAG_FC_DROPPED);
    EXPECT_FALSE(crsfImuPopFrame(&frame));

    // the flag is cleared once a frame got out
    for (int i = 0; i < 80; i++) {
        t += 250;
        crsfImuAccumulate(gyroDps, NULL, t);
    }
    ASSERT_TRUE(crsfImuPopFrame(&frame));
    EXPECT_EQ(9, frame.seq);
    EXPECT_FALSE(frame.flags & CRSF_IMU_FLAG_FC_DROPPED);
}

TEST(TelemetryCrsfImuTest, StateFlags)
{
    DISABLE_ARMING_FLAG(ARMED);
    DISABLE_FLIGHT_MODE(ANGLE_MODE);
    gyroCalibrated = true;
    EXPECT_EQ(0, crsfImuStateFlags());

    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    gyroCalibrated = false;
    EXPECT_EQ(CRSF_IMU_FLAG_ARMED | CRSF_IMU_FLAG_ANGLE_MODE | CRSF_IMU_FLAG_GYRO_NOT_READY, crsfImuStateFlags());

    DISABLE_ARMING_FLAG(ARMED);
    DISABLE_FLIGHT_MODE(ANGLE_MODE);
    gyroCalibrated = true;
}

// ---------------------------------------------------------------------------
// 6. Telemetry ON: IMU frames are sent first, regular frames keep flowing at the reduced rate

TEST(TelemetryCrsfImuTest, OnSendsImuFramesAndKeepsRegularTelemetry)
{
    configureImu(true, 5000, 8, CRSF_IMU_ACCEL_OFF, 5);
    startCrsfTelemetry();
    ASSERT_TRUE(crsfImuActive);
    EXPECT_EQ(5, crsfImuStdTlmDiv());

    const float gyroDps[3] = { 0.0f, 0.0f, 500.0f };
    const float accRaw[3] = { 0.0f, 0.0f, accOneG };
    advanceTo((testTimeUs / 100000 + 1) * 100000);
    runLoops(10000000, gyroDps, accRaw);    // 10 s

    const int imuFrames = countType(CRSF_IMU_DEFAULT_FRAME_TYPE);
    EXPECT_NEAR(250, imuFrames, 1);         // 40 ms per frame
    // regular frames: 4 types, each at 10 Hz / 5 = 2 Hz nominal. The stock scheduler runs on 2ms ticks
    // (26ms slots without V3) and a slot that collides with an IMU frame moves by one more tick.
    EXPECT_NEAR(19, countType(0x1E), 2);
    EXPECT_NEAR(19, countType(0x08), 2);
    EXPECT_NEAR(19, countType(0x21), 2);
    EXPECT_NEAR(19, countType(0x02), 2);
#if defined(USE_CRSF_V3)
    // heartbeats fill the skipped slots: the receiver still gets an FC frame every ~20ms
    EXPECT_NEAR(500 - 4 * 19, countType(0x0B), 25);
    uint32_t lastRegularUs = 0;
    for (const CapturedFrame &f : capturedFrames) {
        if (f.type() == CRSF_IMU_DEFAULT_FRAME_TYPE) {
            continue;
        }
        if (lastRegularUs) {
            EXPECT_LE(f.timeUs - lastRegularUs, 24000u);
        }
        lastRegularUs = f.timeUs;
    }
#else
    EXPECT_EQ(0, countType(0x0B));
#endif

    uint16_t expectedSeq = 0;
    bool first = true;
    uint32_t previousEnd = 0;
    for (const CapturedFrame &f : capturedFrames) {
        expectValidCrsfFrame(f.bytes);
        if (f.type() != CRSF_IMU_DEFAULT_FRAME_TYPE) {
            continue;
        }
        const uint8_t *d = &f.bytes[5];
        EXPECT_EQ(62, f.bytes[1]);
        EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, f.bytes[3]);
        EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, f.bytes[4]);
        EXPECT_EQ(CRSF_IMU_FORMAT_VERSION, d[0]);
        EXPECT_EQ(0, d[1]);
        const uint16_t seq = beU16(&d[2]);
        const uint32_t endTimeUs = beU32(&d[4]);
        if (!first) {
            EXPECT_EQ(expectedSeq, seq);    // nothing dropped: the telemetry task keeps up
            EXPECT_EQ(40000u, endTimeUs - previousEnd);
        }
        first = false;
        expectedSeq = seq + 1;
        previousEnd = endTimeUs;
        EXPECT_EQ(5000, beU16(&d[8]));
        // frame leaves the FC at most one telemetry tick (2ms) after the batch completed
        EXPECT_LE(f.timeUs - endTimeUs, 2000u);
        for (int i = 0; i < 8; i++) {
            EXPECT_NEAR(436.3, be16(&d[10 + i * 6 + 4]), 1.0);
        }
    }

    // turning it off again restores the stock behaviour
    configureImu(false, 5000, 8, CRSF_IMU_ACCEL_OFF, 5);
    initCrsfTelemetry();
    EXPECT_FALSE(crsfImuActive);
    EXPECT_EQ(1, crsfImuStdTlmDiv());
}

TEST(TelemetryCrsfImuTest, DividerOneKeepsRegularRate)
{
    configureImu(true, 10000, 8, CRSF_IMU_ACCEL_OFF, 1);
    startCrsfTelemetry();
    const float gyroDps[3] = { 0.0f, 0.0f, 0.0f };
    const float accRaw[3] = { 0.0f, 0.0f, accOneG };
    advanceTo((testTimeUs / 100000 + 1) * 100000);
    runLoops(10000000, gyroDps, accRaw);

    EXPECT_NEAR(125, countType(CRSF_IMU_DEFAULT_FRAME_TYPE), 1);   // 80 ms per frame
    // the IMU frame takes a telemetry tick now and then, regular frames still ~10 Hz each
    EXPECT_NEAR(96, countType(0x1E), 4);
    EXPECT_NEAR(96, countType(0x08), 4);
}

// STUBS

extern "C" {

// CRSF IMU telemetry dependencies
uint32_t targetPidLooptime;
acc_t acc;
bool gyroIsCalibrationComplete(void) { return gyroCalibrated; }

int16_t debug[DEBUG16_VALUE_COUNT];

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };

uint16_t GPS_distanceToHome;
gpsSolutionData_t gpsSol;

uint32_t micros(void) { return testTimeUs; }
uint32_t microsISR(void) { return micros(); }

bool featureIsEnabled(uint32_t) { return true; }
void beeperConfirmationBeeps(uint8_t) {}

static serialPort_t testSerialPort;
static serialPortConfig_t testSerialPortConfig;

uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
uint32_t serialTxBytesFree(const serialPort_t *) { return 0; }
uint8_t serialRead(serialPort_t *) { return 0; }
void serialWrite(serialPort_t *, uint8_t) {}
void serialWriteBuf(serialPort_t *, const uint8_t *data, int count)
{
    CapturedFrame frame;
    frame.timeUs = testTimeUs;
    frame.bytes.assign(data, data + count);
    capturedFrames.push_back(frame);
}
void serialSetMode(serialPort_t *, portMode_e) {}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return &testSerialPort; }
void closeSerialPort(serialPort_t *) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return &testSerialPortConfig; }

bool telemetryDetermineEnabledState(portSharing_e) { return true; }
bool telemetryCheckRxPortShared(const serialPortConfig_t *, SerialRXType) { return true; }
bool telemetryIsSensorEnabled(sensor_e) { return true; }

portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e) { return PORTSHARING_NOT_SHARED; }

bool airmodeIsEnabled(void) { return false; }

int32_t getAmperage(void) { return 0; }
uint16_t getBatteryVoltage(void) { return 380; }
uint16_t getLegacyBatteryVoltage(void) { return 38; }
uint16_t getBatteryAverageCellVoltage(void) { return 380; }
batteryState_e getBatteryState(void) { return BATTERY_OK; }
uint8_t calculateBatteryPercentageRemaining(void) { return 67; }
int32_t getEstimatedAltitudeCm(void) { return 0; }
int32_t getMAhDrawn(void) { return 0; }

#if !defined(USE_MSP_OVER_TELEMETRY)
bool sendMspReply(uint8_t, mspResponseFnPtr) { return false; }
bool handleMspFrame(uint8_t *, uint8_t, uint8_t *) { return false; }
#endif
bool isBatteryVoltageConfigured(void) { return true; }
bool isAmperageConfigured(void) { return true; }
timeUs_t rxFrameTimeUs(void) { return 0; }

#if defined(USE_MSP_OVER_TELEMETRY)
bool isEepromWriteInProgress(void) { return false; }
mspDescriptor_t mspDescriptorAlloc(void) { return 0; }
mspResult_e mspFcProcessCommand(mspDescriptor_t, mspPacket_t *, mspPacket_t *, mspPostProcessFnPtr *) { return MSP_RESULT_NO_REPLY; }
#endif
#if defined(USE_CRSF_V3)
uint32_t persistentObjectRead(persistentObjectId_e) { return 0; }
void persistentObjectWrite(persistentObjectId_e, uint32_t) {}
void serialSetBaudRate(serialPort_t *, uint32_t) {}
#endif
}
