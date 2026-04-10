//! Boxing Bag Sensor Firmware — BLE + LSM6DS3TR-C accel streaming
//!
//! LED: red=boot, purple=SD init, cyan=SD ok, yellow=IMU init,
//!      blue=advertising, green=connected (streaming)
//! White blink = panic
//!
//! Key gotchas (learned the hard way):
//! - IMU power pin (P1.08) MUST use OutputDrive::HighDrive — Standard drive
//!   isn't enough current for the IMU + I2C pullups. Without HighDrive the
//!   IMU never powers up and I2C stays stuck low.
//! - TWIM init must use blocking_write_read for the init sequence (or async
//!   if interrupt routing works). With SoftDevice, async works when started
//!   after Softdevice::enable().

#![no_std]
#![no_main]

mod lsm6ds3;

use core::mem;

use defmt::{info, warn, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::{Duration, Ticker, Timer};

use nrf_softdevice::ble::advertisement_builder::{
    AdvertisementDataType, Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload,
    ServiceList,
};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};

use boxing_bag_protocol::{ImuPacket, ImuSample, PACKET_SIZE, SAMPLES_PER_PACKET, SENSOR_NAMES, SERVICE_UUID};

use defmt_rtt as _;

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

const SENSOR_ID: u8 = 0;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

#[nrf_softdevice::gatt_service(uuid = "b0b0ba90-0001-4000-8000-000000000000")]
struct ImuService {
    #[characteristic(uuid = "b0b0ba90-0002-4000-8000-000000000000", notify)]
    imu_data: [u8; 20],
}

#[nrf_softdevice::gatt_server]
struct Server {
    imu: ImuService,
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

    // Red = booted
    leds.set(true, false, false);
    Timer::after_millis(300).await;

    // ── Power on IMU ───────────────────────────────────────────────
    // CRITICAL: Must use HighDrive — the GPIO directly supplies VDD/VDDIO
    // to the LSM6DS3TR-C, and Standard drive (~2 mA) isn't enough.
    let _imu_pwr = Output::new(p.P1_08, Level::High, OutputDrive::HighDrive);
    Timer::after_millis(50).await; // wait for IMU to boot

    // Purple = enabling SoftDevice
    leds.set(true, false, true);
    Timer::after_millis(300).await;

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

    // Cyan = SoftDevice OK
    leds.set(false, true, true);
    Timer::after_millis(300).await;

    let server = unwrap!(Server::new(sd));
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // Yellow = IMU init
    leds.set(true, true, false);

    info!("Initializing IMU...");
    let mut twim_config = twim::Config::default();
    twim_config.sda_pullup = true;
    twim_config.scl_pullup = true;
    let twim = Twim::new(p.TWISPI0, Irqs, p.P0_07, p.P0_27, twim_config);
    let mut imu_driver = lsm6ds3::Lsm6ds3::new(twim);

    let imu_ok = match imu_driver.init().await {
        Ok(()) => {
            info!("IMU initialized");
            true
        }
        Err(e) => {
            warn!("IMU init failed: {:?}", e);
            false
        }
    };
    Timer::after_millis(300).await;

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
            let stream_fut = stream_imu_data(&server, &conn, &mut imu_driver);
            futures::pin_mut!(gatt_fut);
            futures::pin_mut!(stream_fut);
            let _ = futures::future::select(gatt_fut, stream_fut).await;
        } else {
            let _ = gatt_server::run(&conn, &server, |_| {}).await;
        }

        info!("BLE disconnected");
    }
}

// ── Accelerometer streaming ────────────────────────────────────────────
async fn stream_imu_data<T: twim::Instance>(
    server: &Server,
    conn: &Connection,
    imu: &mut lsm6ds3::Lsm6ds3<'_, T>,
) {
    let mut seq: u8 = 0;
    let mut sample_buf = [ImuSample::ZERO; SAMPLES_PER_PACKET];
    let mut sample_idx = 0;
    let mut last_packet_time: Option<embassy_time::Instant> = None;
    let mut packet_start_time = embassy_time::Instant::now();

    let mut ticker = Ticker::every(Duration::from_hz(104));

    loop {
        ticker.next().await;

        let sample = match imu.read_imu().await {
            Ok(s) => s,
            Err(e) => {
                warn!("IMU read error: {:?}", e);
                continue;
            }
        };

        if sample_idx == 0 {
            packet_start_time = embassy_time::Instant::now();
        }

        sample_buf[sample_idx] = sample;
        sample_idx += 1;

        if sample_idx >= SAMPLES_PER_PACKET {
            sample_idx = 0;

            let delta_ms = match last_packet_time {
                None => 0u8,
                Some(prev) => {
                    let d = (packet_start_time - prev).as_millis();
                    if d > 63 { 63 } else { d as u8 }
                }
            };
            last_packet_time = Some(packet_start_time);

            let packet = ImuPacket {
                sensor_id: SENSOR_ID,
                sequence: seq,
                time_delta_ms: delta_ms,
                samples: sample_buf,
            };
            seq = seq.wrapping_add(1);

            let encoded = packet.encode();
            // Pad to 20 bytes to match GATT characteristic declaration
            let mut padded = [0u8; 20];
            padded[..encoded.len()].copy_from_slice(&encoded);
            let _ = server.imu.imu_data_notify(conn, &padded);
        }
    }
}
