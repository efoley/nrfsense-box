/// Minimal LSM6DS3TR-C driver over I2C (TWIM)
///
/// Only implements what we need: configure accelerometer ODR + FS,
/// and burst-read 6 bytes of accel output.
use embassy_nrf::twim::{self, Twim};

use boxing_bag_protocol::AccelSample;

const ADDR: u8 = 0x6A;

// Register addresses
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10; // Accel ODR + full-scale
const CTRL2_G: u8 = 0x11; // Gyro control (we disable it)
const CTRL3_C: u8 = 0x12; // Control register 3
const STATUS_REG: u8 = 0x1E;
const OUTX_L_XL: u8 = 0x28; // Accel X low byte (6 bytes follow)

// Expected WHO_AM_I values
const WHO_AM_I_LSM6DS3: u8 = 0x69;
const WHO_AM_I_LSM6DS3TRC: u8 = 0x6A;

pub struct Lsm6ds3<'d, T: twim::Instance> {
    twim: Twim<'d, T>,
}

impl<'d, T: twim::Instance> Lsm6ds3<'d, T> {
    pub fn new(twim: Twim<'d, T>) -> Self {
        Self { twim }
    }

    /// Verify chip identity and configure for 104 Hz, ±4g accelerometer
    pub async fn init(&mut self) -> Result<(), twim::Error> {
        // Check WHO_AM_I
        let id = self.read_reg(WHO_AM_I).await?;
        defmt::info!("LSM6DS3 WHO_AM_I: {:#04x}", id);
        if id != WHO_AM_I_LSM6DS3 && id != WHO_AM_I_LSM6DS3TRC {
            defmt::error!("Unexpected WHO_AM_I: {:#04x}", id);
        }

        // Software reset
        self.write_reg(CTRL3_C, 0x01).await?;
        embassy_time::Timer::after_millis(10).await;

        // Configure accelerometer: ODR = 104 Hz, FS = ±4g
        // CTRL1_XL = 0b0100_10_00 = 0x48
        //   ODR_XL[7:4] = 0100 → 104 Hz
        //   FS_XL[3:2]  = 10   → ±4g
        //   BW_FILT_XL  = 00   → 400 Hz anti-aliasing
        self.write_reg(CTRL1_XL, 0x48).await?;

        // Disable gyroscope to save power
        self.write_reg(CTRL2_G, 0x00).await?;

        defmt::info!("LSM6DS3 initialized: 104 Hz, ±4g");
        Ok(())
    }

    /// Read a single accelerometer sample (blocking on data-ready)
    pub async fn read_accel(&mut self) -> Result<AccelSample, twim::Error> {
        // Wait for accelerometer data ready (bit 0 of STATUS_REG)
        loop {
            let status = self.read_reg(STATUS_REG).await?;
            if status & 0x01 != 0 {
                break;
            }
            embassy_time::Timer::after_micros(100).await;
        }

        // Burst read 6 bytes: XL, XH, YL, YH, ZL, ZH
        let mut buf = [0u8; 6];
        self.read_regs(OUTX_L_XL, &mut buf).await?;

        Ok(AccelSample {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, twim::Error> {
        let mut buf = [0u8; 1];
        self.twim.write_read(ADDR, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), twim::Error> {
        self.twim.write_read(ADDR, &[reg], buf).await
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), twim::Error> {
        self.twim.write(ADDR, &[reg, val]).await
    }
}
