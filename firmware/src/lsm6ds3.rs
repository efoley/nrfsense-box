/// Minimal LSM6DS3TR-C driver over I2C (embassy TWIM async).
/// Reads both accelerometer and gyroscope.
use embassy_nrf::twim::{self, Twim};

use boxing_bag_protocol::ImuSample;

const ADDR: u8 = 0x6A;

const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10; // Accel ODR + full-scale
const CTRL2_G: u8 = 0x11;  // Gyro ODR + full-scale
const CTRL3_C: u8 = 0x12;  // Control register 3
const STATUS_REG: u8 = 0x1E;
const OUTX_L_G: u8 = 0x22;  // Gyro X low byte (6 bytes gyro + 6 bytes accel follow)

const WHO_AM_I_LSM6DS3: u8 = 0x69;
const WHO_AM_I_LSM6DS3TRC: u8 = 0x6A;

pub struct Lsm6ds3<'d, T: twim::Instance> {
    twim: Twim<'d, T>,
}

impl<'d, T: twim::Instance> Lsm6ds3<'d, T> {
    pub fn new(twim: Twim<'d, T>) -> Self {
        Self { twim }
    }

    /// Verify chip identity and configure accel + gyro
    pub async fn init(&mut self) -> Result<(), twim::Error> {
        let mut buf = [0u8; 1];
        self.twim.write_read(ADDR, &[WHO_AM_I], &mut buf).await?;
        defmt::info!("LSM6DS3 WHO_AM_I: {:#04x}", buf[0]);
        if buf[0] != WHO_AM_I_LSM6DS3 && buf[0] != WHO_AM_I_LSM6DS3TRC {
            defmt::error!("Unexpected WHO_AM_I: {:#04x}", buf[0]);
        }

        // Software reset
        self.twim.write(ADDR, &[CTRL3_C, 0x01]).await?;
        embassy_time::Timer::after_millis(20).await;

        // Accelerometer: 104 Hz, ±16g (max range to avoid clipping on impacts)
        // CTRL1_XL = 0b0100_01_00 = 0x44
        //   ODR[7:4]=0100 (104Hz), FS[3:2]=01 (±16g), BW=00 (400Hz AA)
        self.twim.write(ADDR, &[CTRL1_XL, 0x44]).await?;

        // Gyroscope: 104 Hz, ±2000 dps (max range to avoid clipping on impacts)
        // CTRL2_G = 0b0100_11_0_0 = 0x4C
        //   ODR[7:4]=0100 (104Hz), FS[3:2]=11 (±2000 dps)
        self.twim.write(ADDR, &[CTRL2_G, 0x4C]).await?;

        defmt::info!("LSM6DS3 initialized: 104 Hz, ±16g accel, ±2000 dps gyro");
        Ok(())
    }

    /// Read one IMU sample (accel + gyro), waiting for data-ready.
    /// Burst-reads 12 bytes starting at OUTX_L_G (gyro XYZ then accel XYZ).
    pub async fn read_imu(&mut self) -> Result<ImuSample, twim::Error> {
        // Wait for accel data ready (bit 0 of STATUS_REG).
        // Gyro runs at the same ODR so it'll be ready too.
        loop {
            let mut status = [0u8; 1];
            self.twim.write_read(ADDR, &[STATUS_REG], &mut status).await?;
            if status[0] & 0x01 != 0 {
                break;
            }
            embassy_time::Timer::after_micros(100).await;
        }

        // Burst read 12 bytes from OUTX_L_G:
        // bytes 0-5:  gyro  X_L, X_H, Y_L, Y_H, Z_L, Z_H
        // bytes 6-11: accel X_L, X_H, Y_L, Y_H, Z_L, Z_H
        let mut buf = [0u8; 12];
        self.twim.write_read(ADDR, &[OUTX_L_G], &mut buf).await?;

        Ok(ImuSample {
            gx: i16::from_le_bytes([buf[0], buf[1]]),
            gy: i16::from_le_bytes([buf[2], buf[3]]),
            gz: i16::from_le_bytes([buf[4], buf[5]]),
            ax: i16::from_le_bytes([buf[6], buf[7]]),
            ay: i16::from_le_bytes([buf[8], buf[9]]),
            az: i16::from_le_bytes([buf[10], buf[11]]),
        })
    }
}
