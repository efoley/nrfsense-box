#![no_std]

// ── BLE UUIDs ──────────────────────────────────────────────────────────
// Custom 128-bit UUIDs for the boxing bag sensor service
pub const SERVICE_UUID: [u8; 16] = hex_uuid("b0b0ba90-0001-4000-8000-000000000000");
pub const ACCEL_CHAR_UUID: [u8; 16] = hex_uuid("b0b0ba90-0002-4000-8000-000000000000");

// String versions for PC-side BLE libraries
pub const SERVICE_UUID_STR: &str = "b0b0ba90-0001-4000-8000-000000000000";
pub const ACCEL_CHAR_UUID_STR: &str = "b0b0ba90-0002-4000-8000-000000000000";

// ── Packet format ──────────────────────────────────────────────────────
// Each BLE notification carries:
//   [sensor_id: u8] [sequence: u8] [3 × AccelSample(x_lo, x_hi, y_lo, y_hi, z_lo, z_hi)]
// Total: 2 + 3×6 = 20 bytes — fits in default ATT MTU (23 - 3 = 20 payload)
//
// At 104 Hz sample rate with 3 samples/packet → ~35 notifications/sec per sensor.

pub const SAMPLES_PER_PACKET: usize = 3;
pub const PACKET_SIZE: usize = 2 + SAMPLES_PER_PACKET * 6;

#[derive(Clone, Copy, Debug)]
pub struct AccelSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl AccelSample {
    /// Convert raw LSM6DS3 reading to g's.
    /// Assumes ±4g range → sensitivity = 0.122 mg/LSB
    pub fn to_g(&self) -> (f32, f32, f32) {
        const SCALE: f32 = 0.000122;
        (
            self.x as f32 * SCALE,
            self.y as f32 * SCALE,
            self.z as f32 * SCALE,
        )
    }

    pub fn magnitude_g(&self) -> f32 {
        let (x, y, z) = self.to_g();
        // sqrt not available in no_std without libm, so return squared magnitude
        x * x + y * y + z * z
    }
}

#[derive(Clone, Copy)]
pub struct AccelPacket {
    pub sensor_id: u8,
    pub sequence: u8,
    pub samples: [AccelSample; SAMPLES_PER_PACKET],
}

impl AccelPacket {
    pub fn encode(&self) -> [u8; PACKET_SIZE] {
        let mut buf = [0u8; PACKET_SIZE];
        buf[0] = self.sensor_id;
        buf[1] = self.sequence;
        for (i, s) in self.samples.iter().enumerate() {
            let off = 2 + i * 6;
            let x = s.x.to_le_bytes();
            let y = s.y.to_le_bytes();
            let z = s.z.to_le_bytes();
            buf[off] = x[0];
            buf[off + 1] = x[1];
            buf[off + 2] = y[0];
            buf[off + 3] = y[1];
            buf[off + 4] = z[0];
            buf[off + 5] = z[1];
        }
        buf
    }

    pub fn decode(buf: &[u8; PACKET_SIZE]) -> Self {
        let sensor_id = buf[0];
        let sequence = buf[1];
        let mut samples = [AccelSample { x: 0, y: 0, z: 0 }; SAMPLES_PER_PACKET];
        for (i, s) in samples.iter_mut().enumerate() {
            let off = 2 + i * 6;
            s.x = i16::from_le_bytes([buf[off], buf[off + 1]]);
            s.y = i16::from_le_bytes([buf[off + 2], buf[off + 3]]);
            s.z = i16::from_le_bytes([buf[off + 4], buf[off + 5]]);
        }
        AccelPacket {
            sensor_id,
            sequence,
            samples,
        }
    }
}

/// Sensor names for BLE advertisement
pub const SENSOR_NAMES: [&str; 3] = ["BAG_TOP", "BAG_MID", "BAG_BOT"];

// ── Helper ─────────────────────────────────────────────────────────────
/// Parse a UUID string like "b0b0ba90-0001-..." into 16 bytes (little-endian for BLE)
const fn hex_uuid(s: &str) -> [u8; 16] {
    let b = s.as_bytes();
    let mut out = [0u8; 16];
    let mut oi = 15; // BLE UUIDs are little-endian
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
