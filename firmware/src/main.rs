//! Boxing Bag Sensor Firmware — BLE advertising
//!
//! LED: red=boot, purple=SD init, cyan=SD ok, blue=advertising, green=connected
//! White blink = panic

#![no_std]
#![no_main]

use core::mem;

use defmt::{info, warn, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals;
use embassy_time::Timer;

use nrf_softdevice::ble::advertisement_builder::{
    AdvertisementDataType, Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload,
    ServiceList,
};
use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};

mod bitbang_i2c;
mod lsm6ds3;

use embassy_time::Duration;

use boxing_bag_protocol::{AccelPacket, AccelSample, SAMPLES_PER_PACKET, SENSOR_NAMES, SERVICE_UUID};

use defmt_rtt as _;

// Panic handler: blink all LEDs white
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

const SENSOR_ID: u8 = 0;

/// Bit-bang I2C scan — bypasses TWIM entirely, uses raw GPIO.
/// Scans all 7-bit addresses (0x08-0x77) and returns the first that ACKs.
/// Returns Some(addr) if found, None if no device responds.
fn bitbang_i2c_scan() -> Option<u8> {
    unsafe {
        let p0_pin_cnf = 0x5000_0700 as *mut u32;
        let p0_outset = 0x5000_0508 as *mut u32;
        let p0_outclr = 0x5000_050C as *mut u32;
        let p0_in = 0x5000_0510 as *const u32;
        let p0_dirset = 0x5000_0518 as *mut u32;

        const SCL: u32 = 27; // P0.27
        const SDA: u32 = 7;  // P0.07
        const SCL_BIT: u32 = 1 << SCL;
        const SDA_BIT: u32 = 1 << SDA;

        // Configure both as open-drain with pull-up:
        // DIR=1(output), INPUT=0(connect), PULL=3(pullup), DRIVE=6(S0D1)
        let cnf = 1 | (0 << 1) | (3 << 2) | (6 << 8);
        p0_pin_cnf.add(SCL as usize).write_volatile(cnf);
        p0_pin_cnf.add(SDA as usize).write_volatile(cnf);

        // Start with both high (idle)
        p0_outset.write_volatile(SCL_BIT | SDA_BIT);
        cortex_m::asm::delay(5000);

        let delay = || cortex_m::asm::delay(320); // ~5µs at 64MHz → ~100kHz

        let sda_high = || { p0_outset.write_volatile(SDA_BIT); };
        let sda_low = || { p0_outclr.write_volatile(SDA_BIT); };
        let scl_high = || { p0_outset.write_volatile(SCL_BIT); };
        let scl_low = || { p0_outclr.write_volatile(SCL_BIT); };
        let read_sda = || -> bool { (p0_in.read_volatile() & SDA_BIT) != 0 };

        let i2c_start = || {
            sda_high(); delay();
            scl_high(); delay();
            sda_low();  delay(); // SDA falls while SCL high = START
            scl_low();  delay();
        };

        let i2c_stop = || {
            sda_low();  delay();
            scl_high(); delay();
            sda_high(); delay(); // SDA rises while SCL high = STOP
        };

        // Send 8 bits, return ACK (true = ACK received)
        let i2c_write_byte = |byte: u8| -> bool {
            for bit in (0..8).rev() {
                if (byte >> bit) & 1 == 1 {
                    sda_high();
                } else {
                    sda_low();
                }
                delay();
                scl_high(); delay();
                scl_low();  delay();
            }
            // Release SDA for ACK
            sda_high(); delay();
            scl_high(); delay();
            let ack = !read_sda(); // ACK = SDA pulled low by slave
            scl_low();  delay();
            ack
        };

        // Check SDA/SCL lines are high (idle)
        let scl_ok = (p0_in.read_volatile() & SCL_BIT) != 0;
        let sda_ok = (p0_in.read_volatile() & SDA_BIT) != 0;
        defmt::info!("I2C bus state: SCL={} SDA={}", scl_ok as u8, sda_ok as u8);

        if !scl_ok || !sda_ok {
            defmt::warn!("I2C bus stuck! SCL={} SDA={}", scl_ok as u8, sda_ok as u8);
            // Try to recover by clocking SCL
            for _ in 0..18 {
                scl_low(); delay(); delay();
                scl_high(); delay(); delay();
            }
            i2c_stop();
        }

        // Scan addresses 0x08 to 0x77
        let mut found = None;
        for addr in 0x08u8..=0x77 {
            i2c_start();
            let ack = i2c_write_byte((addr << 1) | 0); // write mode
            i2c_stop();

            if ack {
                defmt::info!("I2C device found at address {:#04x}", addr);
                if found.is_none() {
                    found = Some(addr);
                }
            }
        }

        if found.is_none() {
            defmt::warn!("I2C scan: no devices found");
        }

        // Reset pins to input (let TWIM reconfigure)
        p0_pin_cnf.add(SCL as usize).write_volatile(0);
        p0_pin_cnf.add(SDA as usize).write_volatile(0);

        found
    }
}

// No TWIM interrupt needed — using bitbang I2C for the IMU

#[nrf_softdevice::gatt_service(uuid = "b0b0ba90-0001-4000-8000-000000000000")]
struct AccelService {
    #[characteristic(uuid = "b0b0ba90-0002-4000-8000-000000000000", notify)]
    accel_data: [u8; 20],
}

#[nrf_softdevice::gatt_server]
struct Server {
    accel: AccelService,
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

struct Leds<'d> {
    red: Output<'d>,
    green: Output<'d>,
    blue: Output<'d>,
}

impl<'d> Leds<'d> {
    fn set(&mut self, r: bool, g: bool, b: bool) {
        if r { self.red.set_low() } else { self.red.set_high() }
        if g { self.green.set_low() } else { self.green.set_high() }
        if b { self.blue.set_low() } else { self.blue.set_high() }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let mut leds = Leds {
        red: Output::new(p.P0_26, Level::High, OutputDrive::Standard),
        green: Output::new(p.P0_30, Level::High, OutputDrive::Standard),
        blue: Output::new(p.P0_06, Level::High, OutputDrive::Standard),
    };

    // Step 1: Red = booted
    leds.set(true, false, false);
    Timer::after_millis(1000).await;

    // ── Initialize IMU (before SoftDevice) ───────────────────────────
    // Power on IMU
    let _imu_pwr = Output::new(p.P1_08, Level::High, OutputDrive::Standard);
    Timer::after_millis(50).await;

    // Yellow = initializing IMU via bitbang I2C
    leds.set(true, true, false);
    Timer::after_millis(1000).await;

    let mut imu_ok = false;
    let i2c = bitbang_i2c::BitbangI2c::new(27, 7); // SCL=P0.27, SDA=P0.07
    let mut imu = lsm6ds3::Lsm6ds3::new(i2c);

    info!("Initializing IMU (bitbang I2C)...");
    match imu.init() {
        Ok(()) => {
            info!("IMU initialized successfully");
            imu_ok = true;
            leds.set(false, true, false); // green = IMU OK
        }
        Err(e) => {
            warn!("IMU init failed: {}", e);
            leds.set(true, false, false); // red = IMU failed
        }
    }
    Timer::after_millis(1000).await;

    // Step 2: Purple = about to enable SoftDevice
    leds.set(true, false, true);
    Timer::after_millis(1000).await;

    info!("Enabling SoftDevice...");
    let sd = Softdevice::enable(&nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 23 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: SENSOR_NAMES[SENSOR_ID as usize].as_ptr() as *const u8 as *mut u8,
            current_len: SENSOR_NAMES[SENSOR_ID as usize].len() as u16,
            max_len: SENSOR_NAMES[SENSOR_ID as usize].len() as u16,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    });

    // Step 3: Cyan = SoftDevice enabled OK
    leds.set(false, true, true);
    Timer::after_millis(1000).await;

    info!("Creating GATT server...");
    let server = unwrap!(Server::new(sd));
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // Bitbang I2C doesn't use interrupts — works fine with SoftDevice.

    info!("Starting BLE advertising...");

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_128(ServiceList::Complete, &[SERVICE_UUID])
        .build();

    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .raw(AdvertisementDataType::FULL_NAME, b"BAG_SENS")
        .build();

    loop {
        // Blue = advertising
        leds.set(false, false, true);

        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };

        let conn = match peripheral::advertise_connectable(sd, adv, &Default::default()).await {
            Ok(conn) => conn,
            Err(e) => {
                warn!("Advertise error: {:?}", e);
                Timer::after_millis(200).await;
                continue;
            }
        };

        // Green = connected
        leds.set(false, true, false);
        info!("BLE connected!");

        if imu_ok {
            let gatt_fut = gatt_server::run(&conn, &server, |_| {});
            let stream_fut = stream_accel_data(&server, &conn, &mut imu);

            futures::pin_mut!(gatt_fut);
            futures::pin_mut!(stream_fut);

            match futures::future::select(gatt_fut, stream_fut).await {
                futures::future::Either::Left((r, _)) => {
                    info!("GATT server exited: {:?}", r);
                }
                futures::future::Either::Right(((), _)) => {
                    info!("Streaming exited");
                }
            }
        } else {
            let r = gatt_server::run(&conn, &server, |_| {}).await;
            info!("GATT server exited: {:?}", r);
        }

        info!("BLE disconnected, re-advertising...");
    }
}

// ── Accelerometer streaming ────────────────────────────────────────────
async fn stream_accel_data(
    server: &Server,
    conn: &nrf_softdevice::ble::Connection,
    imu: &mut lsm6ds3::Lsm6ds3,
) {
    let mut seq: u8 = 0;
    let mut sample_buf = [AccelSample { x: 0, y: 0, z: 0 }; SAMPLES_PER_PACKET];
    let mut sample_idx = 0;

    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(104));

    loop {
        ticker.next().await;

        // Blocking bitbang read — fast enough at 100kHz (~60µs for 6 bytes)
        let sample = match imu.read_accel() {
            Ok(s) => s,
            Err(_) => continue,
        };

        sample_buf[sample_idx] = sample;
        sample_idx += 1;

        if sample_idx >= SAMPLES_PER_PACKET {
            sample_idx = 0;

            let packet = AccelPacket {
                sensor_id: SENSOR_ID,
                sequence: seq,
                samples: sample_buf,
            };
            seq = seq.wrapping_add(1);

            let encoded = packet.encode();
            let _ = server.accel.accel_data_notify(conn, &encoded);
        }
    }
}
