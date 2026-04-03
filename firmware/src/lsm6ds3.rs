/// Minimal LSM6DS3TR-C driver over bit-bang I2C.
use crate::bitbang_i2c::BitbangI2c;
use boxing_bag_protocol::AccelSample;

const ADDR: u8 = 0x6A;

// Register addresses
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const STATUS_REG: u8 = 0x1E;
const OUTX_L_XL: u8 = 0x28;

const WHO_AM_I_LSM6DS3: u8 = 0x69;
const WHO_AM_I_LSM6DS3TRC: u8 = 0x6A;

pub struct Lsm6ds3 {
    i2c: BitbangI2c,
}

impl Lsm6ds3 {
    pub fn new(i2c: BitbangI2c) -> Self {
        Self { i2c }
    }

    /// Verify chip identity and configure for 104 Hz, ±4g accelerometer
    pub fn init(&mut self) -> Result<(), &'static str> {
        // Check WHO_AM_I
        let id = self.i2c.read_reg(ADDR, WHO_AM_I).ok_or("I2C read failed")?;
        defmt::info!("LSM6DS3 WHO_AM_I: {:#04x}", id);
        if id != WHO_AM_I_LSM6DS3 && id != WHO_AM_I_LSM6DS3TRC {
            defmt::error!("Unexpected WHO_AM_I: {:#04x}", id);
            return Err("Wrong WHO_AM_I");
        }

        // Software reset
        if !self.i2c.write_reg(ADDR, CTRL3_C, 0x01) {
            return Err("Reset write failed");
        }
        cortex_m::asm::delay(64_000_000 / 100); // ~10ms

        // Configure accelerometer: ODR = 104 Hz, FS = ±4g
        if !self.i2c.write_reg(ADDR, CTRL1_XL, 0x48) {
            return Err("CTRL1_XL write failed");
        }

        // Disable gyroscope
        if !self.i2c.write_reg(ADDR, CTRL2_G, 0x00) {
            return Err("CTRL2_G write failed");
        }

        defmt::info!("LSM6DS3 initialized: 104 Hz, ±4g");
        Ok(())
    }

    /// Read a single accelerometer sample (polls for data-ready)
    pub fn read_accel(&mut self) -> Result<AccelSample, &'static str> {
        // Wait for data ready (bit 0 of STATUS_REG)
        for _ in 0..10_000u32 {
            if let Some(status) = self.i2c.read_reg(ADDR, STATUS_REG) {
                if status & 0x01 != 0 {
                    break;
                }
            }
            cortex_m::asm::delay(640); // ~10µs
        }

        // Burst read 6 bytes
        let mut buf = [0u8; 6];
        if !self.i2c.read_regs(ADDR, OUTX_L_XL, &mut buf) {
            return Err("Accel read failed");
        }

        Ok(AccelSample {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }
}
