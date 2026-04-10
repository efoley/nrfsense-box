#![no_std]

// ── BLE UUIDs ──────────────────────────────────────────────────────────
pub const SERVICE_UUID: [u8; 16] = hex_uuid("b0b0ba90-0001-4000-8000-000000000000");
pub const IMU_CHAR_UUID: [u8; 16] = hex_uuid("b0b0ba90-0002-4000-8000-000000000000");

pub const SERVICE_UUID_STR: &str = "b0b0ba90-0001-4000-8000-000000000000";
pub const IMU_CHAR_UUID_STR: &str = "b0b0ba90-0002-4000-8000-000000000000";

// ── Packet format ──────────────────────────────────────────────────────
// Each BLE notification carries (38 bytes, requires ATT MTU ≥ 41):
//
//   byte 0:        sequence (u8, wrapping)
//   byte 1:        bits[7:6] = sensor_id (0-3)
//                  bits[5:0] = time_delta_ms (0-63, saturating)
//   bytes 2-37:    3 × ImuSample (12 bytes each):
//                    accel x/y/z as i16 LE (6 bytes)
//                    gyro  x/y/z as i16 LE (6 bytes)
//
// time_delta_ms is ms elapsed between sample[0] of this packet and
// sample[0] of the previous packet. PC client accumulates deltas.
//
// At 104 Hz with 3 samples/packet → ~35 notifications/sec.

pub const SAMPLES_PER_PACKET: usize = 1;
pub const SAMPLE_SIZE: usize = 12; // 6 accel + 6 gyro
pub const PACKET_SIZE: usize = 2 + SAMPLES_PER_PACKET * SAMPLE_SIZE; // 14
pub const MAX_TIME_DELTA_MS: u8 = 63;

/// Accelerometer scale: ±4g → 0.122 mg/LSB
pub const ACCEL_SCALE: f32 = 0.000122;
/// Gyroscope scale: ±250 dps → 8.75 mdps/LSB
pub const GYRO_SCALE: f32 = 0.00875;

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    /// Accelerometer raw readings (LSM6DS3 units)
    pub ax: i16,
    pub ay: i16,
    pub az: i16,
    /// Gyroscope raw readings (LSM6DS3 units)
    pub gx: i16,
    pub gy: i16,
    pub gz: i16,
}

impl ImuSample {
    pub const ZERO: Self = Self { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };

    /// Convert accel to g's. Assumes ±4g range.
    pub fn accel_g(&self) -> (f32, f32, f32) {
        (
            self.ax as f32 * ACCEL_SCALE,
            self.ay as f32 * ACCEL_SCALE,
            self.az as f32 * ACCEL_SCALE,
        )
    }

    /// Convert gyro to degrees per second. Assumes ±250 dps range.
    pub fn gyro_dps(&self) -> (f32, f32, f32) {
        (
            self.gx as f32 * GYRO_SCALE,
            self.gy as f32 * GYRO_SCALE,
            self.gz as f32 * GYRO_SCALE,
        )
    }

    pub fn accel_magnitude_sq(&self) -> f32 {
        let (x, y, z) = self.accel_g();
        x * x + y * y + z * z
    }
}

#[derive(Clone, Copy)]
pub struct ImuPacket {
    pub sensor_id: u8,
    pub sequence: u8,
    pub time_delta_ms: u8,
    pub samples: [ImuSample; SAMPLES_PER_PACKET],
}

impl ImuPacket {
    pub fn encode(&self) -> [u8; PACKET_SIZE] {
        let mut buf = [0u8; PACKET_SIZE];
        buf[0] = self.sequence;
        buf[1] = ((self.sensor_id & 0x03) << 6) | (self.time_delta_ms & 0x3F);
        for (i, s) in self.samples.iter().enumerate() {
            let off = 2 + i * SAMPLE_SIZE;
            buf[off..off + 2].copy_from_slice(&s.ax.to_le_bytes());
            buf[off + 2..off + 4].copy_from_slice(&s.ay.to_le_bytes());
            buf[off + 4..off + 6].copy_from_slice(&s.az.to_le_bytes());
            buf[off + 6..off + 8].copy_from_slice(&s.gx.to_le_bytes());
            buf[off + 8..off + 10].copy_from_slice(&s.gy.to_le_bytes());
            buf[off + 10..off + 12].copy_from_slice(&s.gz.to_le_bytes());
        }
        buf
    }

    pub fn decode(buf: &[u8; PACKET_SIZE]) -> Self {
        let sequence = buf[0];
        let sensor_id = (buf[1] >> 6) & 0x03;
        let time_delta_ms = buf[1] & 0x3F;
        let mut samples = [ImuSample::ZERO; SAMPLES_PER_PACKET];
        for (i, s) in samples.iter_mut().enumerate() {
            let off = 2 + i * SAMPLE_SIZE;
            s.ax = i16::from_le_bytes([buf[off], buf[off + 1]]);
            s.ay = i16::from_le_bytes([buf[off + 2], buf[off + 3]]);
            s.az = i16::from_le_bytes([buf[off + 4], buf[off + 5]]);
            s.gx = i16::from_le_bytes([buf[off + 6], buf[off + 7]]);
            s.gy = i16::from_le_bytes([buf[off + 8], buf[off + 9]]);
            s.gz = i16::from_le_bytes([buf[off + 10], buf[off + 11]]);
        }
        ImuPacket {
            sensor_id,
            sequence,
            time_delta_ms,
            samples,
        }
    }
}

/// Sensor names for BLE advertisement
pub const SENSOR_NAMES: [&str; 3] = ["BAG_TOP", "BAG_MID", "BAG_BOT"];

// ── Helper ─────────────────────────────────────────────────────────────
const fn hex_uuid(s: &str) -> [u8; 16] {
    let b = s.as_bytes();
    let mut out = [0u8; 16];
    let mut oi = 15;
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'-' {
            i += 1;
            continue;
        }
        let hi = hex_digit(b[i]);
        let lo = hex_digit(b[i + 1]);
        out[oi] = (hi << 4) | lo;
        if oi == 0 {
            break;
        }
        oi -= 1;
        i += 2;
    }
    out
}

const fn hex_digit(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'a'..=b'f' => c - b'a' + 10,
        b'A'..=b'F' => c - b'A' + 10,
        _ => 0,
    }
}
