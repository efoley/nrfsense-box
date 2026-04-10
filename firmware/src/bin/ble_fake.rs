//! BLE advertising with fake accelerometer data.
//! Streams a repeating counter pattern so the pc-client can verify reception.
//! LED: red=boot, blue=advertising, green=connected+streaming

#![no_std]
#![no_main]

use core::mem;
use defmt::{info, warn, unwrap};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_time::{Duration, Ticker, Timer};

use nrf_softdevice::ble::advertisement_builder::{
    AdvertisementDataType, Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload,
    ServiceList,
};
use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};

use boxing_bag_protocol::{AccelPacket, AccelSample, SAMPLES_PER_PACKET, SENSOR_NAMES, SERVICE_UUID};

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

fn set_led(r: bool, g: bool, b: bool) {
    unsafe {
        if r { P0_OUTCLR.write_volatile(1 << 26); } else { P0_OUTSET.write_volatile(1 << 26); }
        if g { P0_OUTCLR.write_volatile(1 << 30); } else { P0_OUTSET.write_volatile(1 << 30); }
        if b { P0_OUTCLR.write_volatile(1 << 6); } else { P0_OUTSET.write_volatile(1 << 6); }
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

    set_led(true, false, false); // red = boot
    Timer::after_millis(500).await;

    let sd = Softdevice::enable(&nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16, rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t { conn_count: 1, event_length: 24 }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 23 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1, periph_role_count: 1,
            central_role_count: 0, central_sec_count: 0,
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

    let server = unwrap!(Server::new(sd));
    unwrap!(spawner.spawn(softdevice_task(sd)));

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_128(ServiceList::Complete, &[SERVICE_UUID])
        .build();
    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .raw(AdvertisementDataType::FULL_NAME, b"BAG_SENS")
        .build();

    info!("BLE fake streamer ready");

    loop {
        set_led(false, false, true); // blue = advertising
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };
        let conn = match peripheral::advertise_connectable(sd, adv, &Default::default()).await {
            Ok(c) => c,
            Err(e) => { warn!("Adv err: {:?}", e); Timer::after_millis(200).await; continue; }
        };

        set_led(false, true, false); // green = connected
        info!("Connected! Streaming fake data...");

        let gatt_fut = gatt_server::run(&conn, &server, |_| {});
        let stream_fut = fake_stream(&server, &conn);
        futures::pin_mut!(gatt_fut);
        futures::pin_mut!(stream_fut);
        let _ = futures::future::select(gatt_fut, stream_fut).await;
        info!("Disconnected");
    }
}

async fn fake_stream(server: &Server, conn: &nrf_softdevice::ble::Connection) {
    let mut seq: u8 = 0;
    let mut counter: i16 = 0;
    let mut last_time: Option<embassy_time::Instant> = None;
    let mut ticker = Ticker::every(Duration::from_hz(104));

    loop {
        ticker.next().await;

        let now = embassy_time::Instant::now();
        let delta_ms = match last_time {
            None => 0u8,
            Some(prev) => {
                let d = (now - prev).as_millis();
                if d > 63 { 63 } else { d as u8 }
            }
        };
        last_time = Some(now);

        let samples = [
            AccelSample { x: counter, y: counter.wrapping_add(1), z: counter.wrapping_add(2) },
            AccelSample { x: counter.wrapping_add(3), y: counter.wrapping_add(4), z: counter.wrapping_add(5) },
            AccelSample { x: counter.wrapping_add(6), y: counter.wrapping_add(7), z: counter.wrapping_add(8) },
        ];
        counter = counter.wrapping_add(9);

        let packet = AccelPacket {
            sensor_id: SENSOR_ID,
            sequence: seq,
            time_delta_ms: delta_ms,
            samples,
        };
        seq = seq.wrapping_add(1);
        let _ = server.accel.accel_data_notify(conn, &packet.encode());
    }
}
