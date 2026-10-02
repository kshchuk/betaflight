#!/usr/bin/env python3
"""Independent reference encoder/decoder for the CRSF IMU frame (format v1)."""
import struct
import sys

SYNC = 0xC8
DEST = 0xEA
ORIG = 0xC8
VERSION = 1
GYRO_SCALE = 1e-4          # rad / LSB
ACC_SCALE = 1e-4           # m/s / LSB (per interval)
ACC_FRAME_SCALE = 1e-3     # m/s / LSB (per frame)

F_ARMED, F_ANGLE, F_GYRO_NOT_READY, F_SATURATED, F_FC_DROPPED = 1, 2, 4, 8, 16
ACC_OFF, ACC_ON, ACC_FRAME = 0, 1, 2


def crc8_d5(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(frame_type, flags, accel_mode, seq, t_end, interval, gyro, accel):
    data = struct.pack('>BBHIH', VERSION, flags | (accel_mode << 5), seq, t_end, interval)
    for i, g in enumerate(gyro):
        data += struct.pack('>3h', *g)
        if accel_mode == ACC_ON:
            data += struct.pack('>3h', *accel[i])
    if accel_mode == ACC_FRAME:
        data += struct.pack('>3h', *accel[0])
    body = bytes([frame_type, DEST, ORIG]) + data
    assert len(body) + 1 <= 62
    return bytes([SYNC, len(body) + 1]) + body + bytes([crc8_d5(body)])


def decode(frame):
    assert frame[0] == SYNC
    ln = frame[1]
    assert len(frame) == ln + 2, 'length'
    body = frame[2:-1]
    assert crc8_d5(body) == frame[-1], 'crc'
    ftype, dest, orig = body[0], body[1], body[2]
    data = body[3:]
    ver, flags, seq, t_end, interval = struct.unpack('>BBHIH', data[:10])
    assert ver == VERSION
    accel_mode = (flags >> 5) & 3
    rest = data[10:]
    if accel_mode == ACC_OFF:
        assert len(rest) % 6 == 0
        n = len(rest) // 6
    elif accel_mode == ACC_ON:
        assert len(rest) % 12 == 0
        n = len(rest) // 12
    else:
        assert (len(rest) - 6) % 6 == 0
        n = (len(rest) - 6) // 6
    gyro, accel, pos = [], [], 0
    for i in range(n):
        gyro.append([v * GYRO_SCALE for v in struct.unpack('>3h', rest[pos:pos + 6])]); pos += 6
        if accel_mode == ACC_ON:
            accel.append([v * ACC_SCALE for v in struct.unpack('>3h', rest[pos:pos + 6])]); pos += 6
    if accel_mode == ACC_FRAME:
        accel.append([v * ACC_FRAME_SCALE for v in struct.unpack('>3h', rest[pos:pos + 6])])
    # end time of interval k (0 = oldest)
    ends = [(t_end - (n - 1 - k) * interval) & 0xFFFFFFFF for k in range(n)]
    return dict(type=ftype, dest=dest, orig=orig, version=ver, flags=flags & 0x1F, accel_mode=accel_mode,
                seq=seq, t_end=t_end, interval=interval, n=n, gyro=gyro, accel=accel, interval_ends=ends)


VECTORS = [
    # 1: gyro only, 8 x 5 ms, armed, yaw left 500 deg/s + small roll/pitch
    dict(frame_type=0x7F, flags=F_ARMED, accel_mode=ACC_OFF, seq=41, t_end=12345678, interval=5000,
         gyro=[[87, -44, 436], [88, -43, 437], [87, -44, 436], [87, -44, 437],
               [88, -44, 436], [87, -43, 436], [87, -44, 437], [88, -44, 436]], accel=[]),
    # 2: gyro + accel per interval, 4 x 2.5 ms, armed + ANGLE, hover, seq wrap value
    dict(frame_type=0x7F, flags=F_ARMED | F_ANGLE, accel_mode=ACC_ON, seq=65535, t_end=4000000000, interval=2500,
         gyro=[[0, 0, 0], [1, 0, -1], [0, -1, 0], [-1, 1, 1]],
         accel=[[3, -2, 245], [2, -1, 245], [3, -2, 246], [2, -2, 245]]),
    # 3: gyro + one accel per frame, 7 x 7.5 ms, disarmed, pitch nose down 200 deg/s
    dict(frame_type=0x7F, flags=0, accel_mode=ACC_FRAME, seq=1000, t_end=60000000, interval=7500,
         gyro=[[0, 262, 0], [0, 262, 0], [0, 261, 0], [0, 262, 0], [0, 262, 0], [0, 262, 0], [0, 261, 0]],
         accel=[[-12, 0, 515]]),
    # 4: saturation, FC dropped a frame, gyro still calibrating
    dict(frame_type=0x7F, flags=F_SATURATED | F_FC_DROPPED | F_GYRO_NOT_READY, accel_mode=ACC_OFF, seq=7,
         t_end=1500000, interval=20000, gyro=[[32767, -32767, 0], [-32767, 32767, 12345]], accel=[]),
    # 5: one interval, alternative frame type 0x80
    dict(frame_type=0x80, flags=F_ARMED, accel_mode=ACC_OFF, seq=0, t_end=0xFFFFFFF0, interval=1000,
         gyro=[[-1, -2, -3]], accel=[]),
]


def main():
    for i, v in enumerate(VECTORS, 1):
        f = encode(**v)
        d = decode(f)
        print(f'V{i} len={len(f)} {f.hex(" ").upper()}')
        if '-v' in sys.argv:
            print('   ', d)


if __name__ == '__main__':
    main()
