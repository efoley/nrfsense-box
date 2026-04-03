//! I2C test — no SoftDevice, no BLE. Just try to talk to the IMU.
//! LED: red=boot, yellow=I2C init, green=WHO_AM_I OK, red blink=failed
//! After init, blinks blue once per successful accel read.

#![no_std]
#![no_main]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::Timer;

use defmt_rtt as _;
// Pull in nrf-softdevice just for its critical-section implementation
use nrf_softdevice as _;

// Panic handler: blink white
const P0_OUTSET: *mut u32 = 0x5000_0508 as *mut u32;
const P0_OUTCLR: *mut u32 = 0x5000_050C as *mut u32;
const P0_PIN_CNF_BASE: *mut u32 = 0x5000_0700 as *mut u32;
const LED_MASK: u32 = (1 << 26) | (1 << 30) | (1 << 6);

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe {
        for pin in [6u32, 26, 30] {
            P0_PIN_CNF_BASE.add(pin as usize).write_volatile(3);
        }
        loop {
            P0_OUTCLR.write_volatile(LED_MASK);
            cortex_m::asm::delay(2_000_000);
            P0_OUTSET.write_volatile(LED_MASK);
            cortex_m::asm::delay(2_000_000);
        }
    }
}

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

fn set_led(r: bool, g: bool, b: bool) {
    unsafe {
        if r { P0_OUTCLR.write_volatile(1 << 26); } else { P0_OUTSET.write_volatile(1 << 26); }
        if g { P0_OUTCLR.write_volatile(1 << 30); } else { P0_OUTSET.write_volatile(1 << 30); }
        if b { P0_OUTCLR.write_volatile(1 << 6); } else { P0_OUTSET.write_volatile(1 << 6); }
    }
}

const IMU_ADDR: u8 = 0x6A;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    // Init LED pins
    let _r = Output::new(p.P0_26, Level::High, OutputDrive::Standard);
    let _g = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    let _b = Output::new(p.P0_06, Level::High, OutputDrive::Standard);

    // Red = boot
    set_led(true, false, false);
    Timer::after_millis(500).await;

    // Power on IMU
    info!("Powering IMU (P1.08)...");
    let _imu_pwr = Output::new(p.P1_08, Level::High, OutputDrive::Standard);
    Timer::after_millis(50).await;

    // Yellow = I2C init
    set_led(true, true, false);
    Timer::after_millis(500).await;

    info!("Creating TWIM...");
    let mut twim_config = twim::Config::default();
    twim_config.sda_pullup = true;
    twim_config.scl_pullup = true;
    let mut twim = Twim::new(p.TWISPI0, Irqs, p.P0_07, p.P0_27, twim_config);

    info!("Reading WHO_AM_I...");
    let mut buf = [0u8; 1];
    match twim.write_read(IMU_ADDR, &[0x0F], &mut buf).await {
        Ok(()) => {
            info!("WHO_AM_I = {:#04x}", buf[0]);
            if buf[0] == 0x6A || buf[0] == 0x69 {
                // Green = success!
                set_led(false, true, false);
                info!("IMU found!");
            } else {
                info!("Unexpected WHO_AM_I value");
                set_led(true, false, true); // purple = wrong ID
            }
        }
        Err(e) => {
            warn!("I2C error: {:?}", e);
            // Red blink = error
            loop {
                set_led(true, false, false);
                Timer::after_millis(300).await;
                set_led(false, false, false);
                Timer::after_millis(300).await;
            }
        }
    }

    Timer::after_millis(1000).await;

    // Configure IMU: 104 Hz, ±4g
    info!("Configuring IMU...");
    let _ = twim.write(IMU_ADDR, &[0x12, 0x01]).await; // reset
    Timer::after_millis(20).await;
    let _ = twim.write(IMU_ADDR, &[0x10, 0x48]).await; // CTRL1_XL
    let _ = twim.write(IMU_ADDR, &[0x11, 0x00]).await; // CTRL2_G off

    info!("Reading accel data continuously...");
    loop {
        Timer::after_millis(100).await; // ~10 Hz for visibility

        let mut data = [0u8; 6];
        match twim.write_read(IMU_ADDR, &[0x28], &mut data).await {
            Ok(()) => {
                let x = i16::from_le_bytes([data[0], data[1]]);
                let y = i16::from_le_bytes([data[2], data[3]]);
                let z = i16::from_le_bytes([data[4], data[5]]);
                info!("accel raw: x={} y={} z={}", x, y, z);
                // Blue blink = successful read
                set_led(false, false, true);
                Timer::after_millis(50).await;
                set_led(false, true, false); // back to green
            }
            Err(e) => {
                warn!("Read error: {:?}", e);
                set_led(true, false, false);
                Timer::after_millis(50).await;
                set_led(false, true, false);
            }
        }
    }
}
