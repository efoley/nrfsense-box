//! Boxing Bag PC Client
//!
//! Scans for BAG_HI/BAG_LO BLE sensors, connects to all found,
//! subscribes to accelerometer notifications, and displays a live terminal
//! dashboard while logging data to CSV.
//!
//! Usage:
//!   cargo run --release
//!   cargo run --release -- --csv output.csv    # custom CSV path
//!   cargo run --release -- --no-tui            # CSV-only, no dashboard

use std::collections::HashMap;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use btleplug::api::{
    Central, CentralEvent, Characteristic, Manager as _, Peripheral as _, ScanFilter,
    WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use chrono::Local;
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use crossterm::ExecutableCommand;
use futures::stream::StreamExt;
use ratatui::prelude::*;
use ratatui::widgets::*;
use tokio::sync::mpsc;
use uuid::Uuid;

use boxing_bag_protocol::{ImuPacket, ImuSample, PACKET_SIZE, SAMPLES_PER_PACKET, SENSOR_NAMES};

// ── BLE UUIDs ──────────────────────────────────────────────────────────
const SERVICE_UUID: Uuid = Uuid::from_u128(0xb0b0ba90_0001_4000_8000_000000000000);
const IMU_CHAR_UUID: Uuid = Uuid::from_u128(0xb0b0ba90_0002_4000_8000_000000000000);

// ── Shared state ───────────────────────────────────────────────────────
#[derive(Clone, Debug)]
struct SensorState {
    name: String,
    last_sample: ImuSample,
    last_g: (f32, f32, f32),
    peak_magnitude: f32,
    packets_received: u64,
    last_seq: u8,
    dropped_packets: u64,
    last_update: Instant,
    samples_per_sec: f32,
    magnitude_history: Vec<f32>,
}

impl SensorState {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            last_sample: ImuSample::ZERO,
            last_g: (0.0, 0.0, 0.0),
            peak_magnitude: 0.0,
            packets_received: 0,
            last_seq: 0,
            dropped_packets: 0,
            last_update: Instant::now(),
            samples_per_sec: 0.0,
            magnitude_history: Vec::with_capacity(120),
        }
    }
}

type SharedState = Arc<Mutex<HashMap<u8, SensorState>>>;

// ── CSV writer ─────────────────────────────────────────────────────────
struct CsvLogger {
    writer: csv::Writer<std::fs::File>,
}

impl CsvLogger {
    fn new(path: &str) -> Result<Self> {
        let writer = csv::Writer::from_path(path)
            .with_context(|| format!("Failed to create CSV: {}", path))?;
        let mut logger = Self { writer };
        logger
            .writer
            .write_record(["timestamp", "device_ms", "sensor_id", "sensor_name", "seq",
                "ax_raw", "ay_raw", "az_raw", "ax_g", "ay_g", "az_g", "accel_mag_g",
                "gx_raw", "gy_raw", "gz_raw", "gx_dps", "gy_dps", "gz_dps"])
            .ok();
        logger.writer.flush().ok();
        Ok(logger)
    }

    /// Log a sample with device-side timing.
    /// `device_time_ms` is the reconstructed absolute device time for this sample.
    fn log_sample(&mut self, sensor_id: u8, seq: u8, sample: &ImuSample, device_time_ms: f64) {
        let (ax, ay, az) = sample.accel_g();
        let accel_mag = (ax * ax + ay * ay + az * az).sqrt();
        let (gx, gy, gz) = sample.gyro_dps();
        let name = if (sensor_id as usize) < SENSOR_NAMES.len() {
            SENSOR_NAMES[sensor_id as usize]
        } else {
            "UNKNOWN"
        };

        self.writer
            .write_record(&[
                Local::now().format("%Y-%m-%d %H:%M:%S%.3f").to_string(),
                format!("{:.1}", device_time_ms),
                sensor_id.to_string(),
                name.to_string(),
                seq.to_string(),
                sample.ax.to_string(),
                sample.ay.to_string(),
                sample.az.to_string(),
                format!("{:.4}", ax),
                format!("{:.4}", ay),
                format!("{:.4}", az),
                format!("{:.4}", accel_mag),
                sample.gx.to_string(),
                sample.gy.to_string(),
                sample.gz.to_string(),
                format!("{:.2}", gx),
                format!("{:.2}", gy),
                format!("{:.2}", gz),
            ])
            .ok();
        self.writer.flush().ok();
    }
}

// ── BLE discovery & connection ─────────────────────────────────────────
async fn get_adapter() -> Result<Adapter> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    adapters
        .into_iter()
        .next()
        .context("No Bluetooth adapters found")
}

async fn find_sensors(adapter: &Adapter, timeout_secs: u64) -> Result<Vec<Peripheral>> {
    println!("Scanning for boxing bag sensors ({timeout_secs}s)...");
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(Duration::from_secs(timeout_secs)).await;

    let mut sensors = Vec::new();
    for p in adapter.peripherals().await? {
        if let Some(props) = p.properties().await? {
            let name = props.local_name.unwrap_or_default();
            if SENSOR_NAMES.iter().any(|sn| name.contains(sn)) || name.contains("BAG_SENS") {
                println!("  Found: {} ({})", name, props.address);
                sensors.push(p);
            }
        }
    }

    adapter.stop_scan().await?;
    println!("Found {} sensor(s)", sensors.len());
    Ok(sensors)
}

async fn connect_and_subscribe(
    peripheral: &Peripheral,
    tx: mpsc::UnboundedSender<(u8, ImuPacket)>,
) -> Result<()> {
    peripheral.connect().await.context("BLE connect failed")?;
    peripheral.discover_services().await?;

    let chars = peripheral.characteristics();
    let imu_char = chars
        .iter()
        .find(|c| c.uuid == IMU_CHAR_UUID)
        .context("IMU characteristic not found")?
        .clone();

    peripheral.subscribe(&imu_char).await?;

    let mut notifs = peripheral.notifications().await?;
    let tx = tx.clone();

    tokio::spawn(async move {
        let mut logged_size = false;
        while let Some(data) = notifs.next().await {
            if !logged_size {
                eprintln!("  First notification: {} bytes (expected {})", data.value.len(), PACKET_SIZE);
                logged_size = true;
            }
            if data.value.len() >= PACKET_SIZE {
                let mut buf = [0u8; PACKET_SIZE];
                buf.copy_from_slice(&data.value[..PACKET_SIZE]);
                let packet = ImuPacket::decode(&buf);
                let _ = tx.send((packet.sensor_id, packet));
            }
        }
    });

    Ok(())
}

// ── Terminal UI ────────────────────────────────────────────────────────
fn draw_dashboard(frame: &mut Frame, state: &HashMap<u8, SensorState>) {
    let area = frame.area();

    // Title
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // title
            Constraint::Min(0),   // sensor panels
            Constraint::Length(1), // footer
        ])
        .split(area);

    let title_block = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .title(" Boxing Bag Sensor Dashboard ")
        .title_alignment(Alignment::Center);
    let title = Paragraph::new(format!(
        " {} sensor(s) connected  |  Press 'q' to quit  |  Press 'r' to reset peaks",
        state.len()
    ))
    .block(title_block)
    .alignment(Alignment::Center);
    frame.render_widget(title, chunks[0]);

    // One panel per sensor
    let sensor_count = state.len().max(1);
    let sensor_constraints: Vec<Constraint> =
        (0..sensor_count).map(|_| Constraint::Ratio(1, sensor_count as u32)).collect();
    let sensor_areas = Layout::default()
        .direction(Direction::Horizontal)
        .constraints(sensor_constraints)
        .split(chunks[1]);

    let mut sorted_ids: Vec<u8> = state.keys().cloned().collect();
    sorted_ids.sort();

    for (i, &id) in sorted_ids.iter().enumerate() {
        if i >= sensor_areas.len() {
            break;
        }
        if let Some(s) = state.get(&id) {
            let inner = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(7), // stats
                    Constraint::Min(5),    // sparkline
                ])
                .split(sensor_areas[i]);

            // Stats
            let block = Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded)
                .title(format!(" {} (ID: {}) ", s.name, id));

            let (gx, gy, gz) = s.last_g;
            let mag = (gx * gx + gy * gy + gz * gz).sqrt();
            let stats_text = vec![
                Line::from(format!("  X: {:+8.3}g  Y: {:+8.3}g  Z: {:+8.3}g", gx, gy, gz)),
                Line::from(format!("  |G|: {:6.3}g   Peak: {:6.3}g", mag, s.peak_magnitude)),
                Line::from(format!(
                    "  Packets: {}   Dropped: {}   Rate: {:.0} samp/s",
                    s.packets_received, s.dropped_packets, s.samples_per_sec
                )),
                Line::from(""),
                Line::from(format!("  Accel raw: x={:+6} y={:+6} z={:+6}", s.last_sample.ax, s.last_sample.ay, s.last_sample.az)),
            ];
            let stats = Paragraph::new(stats_text).block(block);
            frame.render_widget(stats, inner[0]);

            // Sparkline of magnitude history
            let spark_block = Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded)
                .title(" Magnitude (g) ");
            if !s.magnitude_history.is_empty() {
                // Scale to u64 for sparkline (multiply by 100)
                let data: Vec<u64> = s
                    .magnitude_history
                    .iter()
                    .map(|v| (v * 100.0) as u64)
                    .collect();
                let sparkline = Sparkline::default()
                    .block(spark_block)
                    .data(&data)
                    .style(Style::default().fg(Color::Cyan));
                frame.render_widget(sparkline, inner[1]);
            } else {
                let empty = Paragraph::new("  Waiting for data...")
                    .block(spark_block);
                frame.render_widget(empty, inner[1]);
            }
        }
    }

    // Footer
    let footer = Paragraph::new(" CSV logging active | Data: 104 Hz × 2 sensors ")
        .style(Style::default().fg(Color::DarkGray))
        .alignment(Alignment::Center);
    frame.render_widget(footer, chunks[2]);
}

// ── Main ───────────────────────────────────────────────────────────────
#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let csv_path = args
        .iter()
        .position(|a| a == "--csv")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("boxing_bag_data.csv");
    let no_tui = args.iter().any(|a| a == "--no-tui");

    // Open CSV logger
    let csv_logger = Arc::new(Mutex::new(CsvLogger::new(csv_path)?));
    println!("Logging to: {}", csv_path);

    // Shared sensor state
    let state: SharedState = Arc::new(Mutex::new(HashMap::new()));

    // BLE packet channel
    let (tx, mut rx) = mpsc::unbounded_channel::<(u8, ImuPacket)>();

    // Discover and connect to sensors
    let adapter = get_adapter().await?;
    let sensors = find_sensors(&adapter, 5).await?;

    if sensors.is_empty() {
        println!("No sensors found! Make sure firmware is running and sensors are advertising.");
        println!("Tip: Use 'nRF Connect' app on your phone to verify sensors are visible.");
        return Ok(());
    }

    for sensor in &sensors {
        let name = sensor
            .properties()
            .await?
            .and_then(|p| p.local_name)
            .unwrap_or_else(|| "Unknown".into());
        println!("Connecting to {}...", name);
        if let Err(e) = connect_and_subscribe(sensor, tx.clone()).await {
            eprintln!("  Failed to connect to {}: {}", name, e);
        } else {
            println!("  Connected and subscribed!");
        }
    }

    // Spawn packet processing task
    let state_clone = state.clone();
    let csv_clone = csv_logger.clone();
    let rate_window = Duration::from_secs(2);

    tokio::spawn(async move {
        let mut rate_counters: HashMap<u8, (Instant, u64)> = HashMap::new();
        // Per-sensor accumulated device time (ms) from delta encoding
        let mut device_times: HashMap<u8, f64> = HashMap::new();
        // Inter-sample period in ms (1000 / 104 Hz ≈ 9.615 ms)
        let sample_period_ms = 1000.0 / 104.0; // ~9.615 ms between IMU samples

        while let Some((sensor_id, packet)) = rx.recv().await {
            // Accumulate device-side time from delta encoding
            let device_ms = device_times.entry(sensor_id).or_insert(0.0);
            *device_ms += packet.time_delta_ms as f64;

            // Log each sample with interpolated device timestamp
            if let Ok(mut logger) = csv_clone.lock() {
                for (i, sample) in packet.samples.iter().enumerate() {
                    let sample_time = *device_ms + i as f64 * sample_period_ms;
                    logger.log_sample(sensor_id, packet.sequence, sample, sample_time);
                }
            }

            // Update shared state
            if let Ok(mut st) = state_clone.lock() {
                let entry = st
                    .entry(sensor_id)
                    .or_insert_with(|| {
                        let name = if (sensor_id as usize) < SENSOR_NAMES.len() {
                            SENSOR_NAMES[sensor_id as usize]
                        } else {
                            "UNKNOWN"
                        };
                        SensorState::new(name)
                    });

                // Detect dropped packets via sequence gap
                let expected = entry.last_seq.wrapping_add(1);
                if entry.packets_received > 0 && packet.sequence != expected {
                    let gap = packet.sequence.wrapping_sub(expected) as u64;
                    entry.dropped_packets += gap;
                }
                entry.last_seq = packet.sequence;
                entry.packets_received += 1;

                // Update last sample (use final sample in packet)
                let last = packet.samples[SAMPLES_PER_PACKET - 1];
                entry.last_sample = last;
                entry.last_g = last.accel_g();

                let (gx, gy, gz) = entry.last_g;
                let mag = (gx * gx + gy * gy + gz * gz).sqrt();
                if mag > entry.peak_magnitude {
                    entry.peak_magnitude = mag;
                }

                // Magnitude history for sparkline
                entry.magnitude_history.push(mag);
                if entry.magnitude_history.len() > 120 {
                    entry.magnitude_history.remove(0);
                }

                entry.last_update = Instant::now();

                // Calculate samples/sec
                let counter = rate_counters
                    .entry(sensor_id)
                    .or_insert_with(|| (Instant::now(), 0));
                counter.1 += SAMPLES_PER_PACKET as u64;
                let elapsed = counter.0.elapsed();
                if elapsed >= rate_window {
                    entry.samples_per_sec = counter.1 as f32 / elapsed.as_secs_f32();
                    *counter = (Instant::now(), 0);
                }
            }
        }
    });

    // ── UI loop or headless ────────────────────────────────────────
    if no_tui {
        println!("Running headless (--no-tui). Press Ctrl+C to stop.");
        tokio::signal::ctrl_c().await?;
    } else {
        // Setup terminal
        enable_raw_mode()?;
        io::stdout().execute(EnterAlternateScreen)?;
        let mut terminal = Terminal::new(CrosstermBackend::new(io::stdout()))?;

        loop {
            // Draw
            {
                let st = state.lock().unwrap();
                terminal.draw(|f| draw_dashboard(f, &st))?;
            }

            // Handle input (non-blocking, 50ms poll)
            if event::poll(Duration::from_millis(50))? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press {
                        match key.code {
                            KeyCode::Char('q') | KeyCode::Esc => break,
                            KeyCode::Char('r') => {
                                // Reset peaks
                                if let Ok(mut st) = state.lock() {
                                    for s in st.values_mut() {
                                        s.peak_magnitude = 0.0;
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                }
            }
        }

        // Restore terminal
        disable_raw_mode()?;
        io::stdout().execute(LeaveAlternateScreen)?;
    }

    // Disconnect sensors
    for sensor in &sensors {
        let _ = sensor.disconnect().await;
    }

    println!("Data saved to: {}", csv_path);
    Ok(())
}
