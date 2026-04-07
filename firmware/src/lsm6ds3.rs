/// Minimal LSM6DS3TR-C driver over I2C (embassy TWIM async).
use embassy_nrf::twim::{self, Twim};

use boxing_bag_protocol::AccelSample;

const ADDR: u8 = 0x6A;

// Register addresses
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const STATUS_REG: u8 = 0x1E;
const OUTX_L_XL: u8 = 0x28;

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
        let mut buf = [0u8; 1];
        self.twim.write_read(ADDR, &[WHO_AM_I], &mut buf).await?;
        defmt::info!("LSM6DS3 WHO_AM_I: {:#04x}", buf[0]);
        if buf[0] != WHO_AM_I_LSM6DS3 && buf[0] != WHO_AM_I_LSM6DS3TRC {
            defmt::error!("Unexpected WHO_AM_I: {:#04x}", buf[0]);
        }

        // Software reset
        self.twim.write(ADDR, &[CTRL3_C, 0x01]).await?;
        embassy_time::Timer::after_millis(20).await;

        // Configure accelerometer: ODR = 104 Hz, FS = ±4g
        // CTRL1_XL = 0b0100_10_00 = 0x48
        self.twim.write(ADDR, &[CTRL1_XL, 0x48]).await?;

        // Disable gyroscope to save power
        self.twim.write(ADDR, &[CTRL2_G, 0x00]).await?;

        defmt::info!("LSM6DS3 initialized: 104 Hz, ±4g");
        Ok(())
    }

    /// Read a single accelerometer sample (waits for data-ready)
    pub async fn read_accel(&mut self) -> Result<AccelSample, twim::Error> {
        // Wait for accelerometer data ready (bit 0 of STATUS_REG)
        loop {
            let mut status = [0u8; 1];
            self.twim.write_read(ADDR, &[STATUS_REG], &mut status).await?;
            if status[0] & 0x01 != 0 {
                break;
            }
            embassy_time::Timer::after_micros(100).await;
        }

        // Burst read 6 bytes: XL, XH, YL, YH, ZL, ZH
        let mut buf = [0u8; 6];
        self.twim.write_read(ADDR, &[OUTX_L_XL], &mut buf).await?;

        Ok(AccelSample {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }
}
