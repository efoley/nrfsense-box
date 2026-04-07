//! I2C test with USB serial logging and TWIM register monitoring.
//!
//! No SoftDevice — uses HardwareVbusDetect for USB.
//! Spawns a monitor task that polls TWIM0 hardware registers and reports
//! state via USB serial. Runs the I2C transaction in main and logs result.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::pac;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver as UsbDriver;
use embassy_nrf::{bind_interrupts, peripherals};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::UsbDevice;
use heapless::String;

use defmt_rtt as _;
// Pull in nrf-softdevice for its critical-section-impl (we don't actually
// enable the SoftDevice in this binary)
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
    USBD => embassy_nrf::usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => embassy_nrf::usb::vbus_detect::InterruptHandler;
});

const IMU_ADDR: u8 = 0x6A;

type UsbD = UsbDriver<'static, peripherals::USBD, HardwareVbusDetect>;

// Channel for log messages from any task to the USB sender task
static LOG_CHAN: Channel<CriticalSectionRawMutex, String<128>, 16> = Channel::new();

// Boot diagnostics — captured once at startup, included in every monitor message
static BOOT_SDA: AtomicU32 = AtomicU32::new(99);
static BOOT_SCL: AtomicU32 = AtomicU32::new(99);

fn set_led(r: bool, g: bool, b: bool) {
    unsafe {
        if r { P0_OUTCLR.write_volatile(1 << 26); } else { P0_OUTSET.write_volatile(1 << 26); }
        if g { P0_OUTCLR.write_volatile(1 << 30); } else { P0_OUTSET.write_volatile(1 << 30); }
        if b { P0_OUTCLR.write_volatile(1 << 6); } else { P0_OUTSET.write_volatile(1 << 6); }
    }
}

/// Send a log message via the channel (non-blocking; drops on full)
fn log(msg: &str) {
    let mut s: String<128> = String::new();
    let _ = s.push_str(msg);
    let _ = LOG_CHAN.try_send(s);
}

#[embassy_executor::task]
async fn usb_run_task(mut usb: UsbDevice<'static, UsbD>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn usb_log_task(mut class: CdcAcmClass<'static, UsbD>) {
    loop {
        class.wait_connection().await;
        let _ = class.write_packet(b"=== i2c-test USB serial ready ===\r\n").await;
        loop {
            // Drain channel and write
            let msg = LOG_CHAN.receive().await;
            // Write in chunks of <=64 bytes
            let bytes = msg.as_bytes();
            let mut start = 0;
            while start < bytes.len() {
                let end = (start + 60).min(bytes.len());
                if class.write_packet(&bytes[start..end]).await.is_err() {
                    break;
                }
                start = end;
            }
            let _ = class.write_packet(b"\r\n").await;
        }
    }
}

/// Periodically dump TWIM0 hardware register state via USB serial
#[embassy_executor::task]
async fn twim_monitor_task() {
    loop {
        Timer::after_millis(500).await;

        unsafe {
            let twim0 = pac::TWIM0;
            let events_stopped = twim0.events_stopped().read();
            let events_error = twim0.events_error().read();
            let events_lasttx = twim0.events_lasttx().read();
            let events_lastrx = twim0.events_lastrx().read();
            let events_txstarted = twim0.events_txstarted().read();
            let events_rxstarted = twim0.events_rxstarted().read();
            let events_suspended = twim0.events_suspended().read();
            let errorsrc = twim0.errorsrc().read();
            let inten = twim0.inten().read();
            let enable = twim0.enable().read();
            let address = twim0.address().read().0;

            // Read live SDA/SCL state from GPIO. TWIM disconnects input buffers,
            // so we need to temporarily connect them.
            let p0_in = 0x5000_0510 as *const u32;
            let p0_pin_cnf = 0x5000_0700 as *mut u32;
            let scl_cnf = p0_pin_cnf.add(27).read_volatile();
            let sda_cnf = p0_pin_cnf.add(7).read_volatile();
            p0_pin_cnf.add(27).write_volatile(scl_cnf & !(1 << 1));
            p0_pin_cnf.add(7).write_volatile(sda_cnf & !(1 << 1));
            cortex_m::asm::delay(100);
            let in_val = p0_in.read_volatile();
            let sda_now = (in_val >> 7) & 1;
            let scl_now = (in_val >> 27) & 1;
            // Restore
            p0_pin_cnf.add(27).write_volatile(scl_cnf);
            p0_pin_cnf.add(7).write_volatile(sda_cnf);

            // Use PAC for P1 access
            let p1 = pac::P1;
            let imu_pwr_out = (p1.out().read().0 >> 8) & 1;
            let imu_pwr_dir = (p1.dir().read().0 >> 8) & 1;
            let cnf = p1.pin_cnf(8);
            let saved_cnf = cnf.read();
            let mut new_cnf = saved_cnf;
            new_cnf.set_input(pac::gpio::vals::Input::CONNECT);
            cnf.write_value(new_cnf);
            cortex_m::asm::delay(100);
            let imu_pwr_in = (p1.in_().read().0 >> 8) & 1;
            cnf.write_value(saved_cnf);
            let imu_pwr_cnf = saved_cnf.0;

            let boot_sda = BOOT_SDA.load(Ordering::Relaxed);
            let boot_scl = BOOT_SCL.load(Ordering::Relaxed);

            let mut s: String<128> = String::new();
            let _ = write!(
                s,
                "T:en={:?} addr={:02x} txs={} esrc={:x} sda={} scl={} pwr o={} i={} d={} cnf={:x}",
                enable.0, address, events_txstarted, errorsrc.0,
                sda_now, scl_now, imu_pwr_out, imu_pwr_in, imu_pwr_dir, imu_pwr_cnf,
            );
            let _ = LOG_CHAN.try_send(s);
        }
    }
}

#[embassy_executor::task]
async fn i2c_task(mut twim: Twim<'static, peripherals::TWISPI0>) {
    Timer::after_millis(2000).await; // give USB time to connect

    log("Starting I2C transaction (async)...");

    let mut buf = [0u8; 1];
    match twim.write_read(IMU_ADDR, &[0x0F], &mut buf).await {
        Ok(()) => {
            let mut s: String<128> = String::new();
            let _ = write!(s, "WHO_AM_I = 0x{:02x}", buf[0]);
            let _ = LOG_CHAN.try_send(s);
            if buf[0] == 0x6A || buf[0] == 0x69 {
                set_led(false, true, false); // green
                log("IMU FOUND!");
            } else {
                set_led(true, false, true); // purple
                log("Unexpected WHO_AM_I");
            }
        }
        Err(e) => {
            set_led(true, false, false);
            let mut s: String<128> = String::new();
            let _ = write!(s, "I2C error: {:?}", e);
            let _ = LOG_CHAN.try_send(s);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    // ── Set up USB CDC serial ──────────────────────────────────────
    let vbus = HardwareVbusDetect::new(Irqs);
    let usb_driver = UsbDriver::new(p.USBD, Irqs, vbus);

    // Use static muts for buffers — no StaticCell so warm reset is safe
    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 256] = [0; 256];
    static mut MSOS_DESC: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];
    static mut CDC_STATE: Option<CdcState> = None;

    let mut usb_config = embassy_usb::Config::new(0x2886, 0x8045);
    usb_config.manufacturer = Some("NRFSense");
    usb_config.product = Some("i2c-test");
    usb_config.serial_number = Some("001");

    let cdc_state = unsafe {
        CDC_STATE = Some(CdcState::new());
        CDC_STATE.as_mut().unwrap()
    };

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        usb_config,
        unsafe { &mut *core::ptr::addr_of_mut!(CONFIG_DESC) },
        unsafe { &mut *core::ptr::addr_of_mut!(BOS_DESC) },
        unsafe { &mut *core::ptr::addr_of_mut!(MSOS_DESC) },
        unsafe { &mut *core::ptr::addr_of_mut!(CONTROL_BUF) },
    );

    let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let usb = builder.build();

    spawner.spawn(usb_run_task(usb)).unwrap();
    spawner.spawn(usb_log_task(class)).unwrap();
    spawner.spawn(twim_monitor_task()).unwrap();

    info!("Boot done, USB starting");

    // Power on IMU using HighDrive (10mA) in case Standard isn't enough.
    let _imu_pwr = Output::new(p.P1_08, Level::High, OutputDrive::HighDrive);
    Timer::after_millis(100).await;

    log("IMU power on (HighDrive)");

    // Yellow = I2C init
    set_led(true, true, false);

    // I2C bus recovery: configure both as input with pullup, check state,
    // then manually toggle SCL 18 times in case the IMU is holding SDA low.
    log("Doing I2C bus recovery...");
    unsafe {
        let p0_pin_cnf = 0x5000_0700 as *mut u32;
        let p0_outset = 0x5000_0508 as *mut u32;
        let p0_outclr = 0x5000_050C as *mut u32;
        let p0_in = 0x5000_0510 as *const u32;

        // First just check state (both as input with pullup)
        let cnf_in = 0 | (0 << 1) | (3 << 2);
        p0_pin_cnf.add(27).write_volatile(cnf_in);
        p0_pin_cnf.add(7).write_volatile(cnf_in);
        cortex_m::asm::delay(64_000); // 1ms for pullups to settle

        let in_val = p0_in.read_volatile();
        let sda_initial = (in_val >> 7) & 1;
        let scl_initial = (in_val >> 27) & 1;
        BOOT_SDA.store(sda_initial, Ordering::Relaxed);
        BOOT_SCL.store(scl_initial, Ordering::Relaxed);

        let mut s: String<128> = String::new();
        let _ = write!(s, "Pre-recovery SDA={} SCL={}", sda_initial, scl_initial);
        let _ = LOG_CHAN.try_send(s);

        // Now make SCL an output and toggle 18 times
        let cnf_out = 1 | (0 << 1) | (3 << 2) | (6 << 8);
        p0_pin_cnf.add(27).write_volatile(cnf_out);
        p0_outset.write_volatile(1 << 27);
        cortex_m::asm::delay(5000);
        for _ in 0..18 {
            p0_outclr.write_volatile(1 << 27);
            cortex_m::asm::delay(640);
            p0_outset.write_volatile(1 << 27);
            cortex_m::asm::delay(640);
        }

        // Make SCL input again, check state
        p0_pin_cnf.add(27).write_volatile(cnf_in);
        cortex_m::asm::delay(64_000);
        let in_val = p0_in.read_volatile();
        let sda_after = (in_val >> 7) & 1;
        let scl_after = (in_val >> 27) & 1;

        let mut s: String<128> = String::new();
        let _ = write!(s, "Post-recovery SDA={} SCL={}", sda_after, scl_after);
        let _ = LOG_CHAN.try_send(s);

        // Reset to fully disconnected so TWIM can take pins
        p0_pin_cnf.add(27).write_volatile(0);
        p0_pin_cnf.add(7).write_volatile(0);
    }

    info!("Creating TWIM...");
    let mut twim_config = twim::Config::default();
    twim_config.sda_pullup = true;
    twim_config.scl_pullup = true;
    twim_config.frequency = twim::Frequency::K100;
    let twim = Twim::new(p.TWISPI0, Irqs, p.P0_07, p.P0_27, twim_config);

    spawner.spawn(i2c_task(twim)).unwrap();

    // Main loop just heartbeats — actual work is in tasks
    loop {
        Timer::after_millis(5000).await;
        log("heartbeat");
    }
}
