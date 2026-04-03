# CLAUDE.md — Boxing Bag Sensor System

## Project Overview

BLE-based accelerometer streaming system: 3× Seeed XIAO nRF52840 Sense boards
attached to a boxing heavy bag stream LSM6DS3TR-C IMU data at 104 Hz to a Rust
PC client that displays a live terminal dashboard and logs CSV.

## Repository Structure

```
boxing-bag-sensors/
├── firmware/        # Embedded Rust — Embassy-nrf + nrf-softdevice (BLE peripheral)
├── pc-client/       # Desktop Rust — btleplug + ratatui (BLE central + dashboard)
└── protocol/        # Shared no_std crate — packet format, UUIDs, constants
```

## Build Commands

### Firmware (embedded, cross-compiled)

```bash
cd firmware

# Build only
cargo build --release

# Build + flash via UF2 (double-tap reset on XIAO first!)
./flash_uf2.sh

# Build + flash via SWD probe (uncomment runner in .cargo/config.toml first)
# cargo run --release
```

- Target: `thumbv7em-none-eabihf` (set in `.cargo/config.toml`)
- UF2 flashing: `./flash_uf2.sh` (default, no probe needed)
- SWD flashing: `cargo run --release` (requires probe-rs + debug probe, gives RTT logs)
- Requires: `rustup target add thumbv7em-none-eabihf`
- Requires: `cargo install cargo-binutils`, `rustup component add llvm-tools`
- Requires: `uf2conv.py` in `firmware/tools/` (see README for download)
- The factory Adafruit bootloader already includes SoftDevice S140 v7.3.0 — no separate SD flash needed

### PC Client (native)

```bash
cd pc-client
cargo build --release
cargo run --release
cargo run --release -- --csv output.csv --no-tui
```

- Linux BLE requires either root or `setcap 'cap_net_raw,cap_net_admin+eip'` on the binary

### Protocol (library, no_std)

```bash
cd protocol
cargo build                          # host check
cargo build --target thumbv7em-none-eabihf  # embedded check
```

## Hardware Details — XIAO nRF52840 Sense

| Signal       | Pin   | Notes                              |
|-------------|-------|------------------------------------|
| IMU I2C SDA | P0.07 | Internal, not on headers           |
| IMU I2C SCL | P0.27 | Internal, not on headers           |
| IMU Power   | P1.08 | GPIO must drive HIGH to power IMU  |
| IMU INT1    | P0.11 | Directly interrupt, active high    |
| IMU Addr    | 0x6A  | LSM6DS3TR-C default                |
| LED Red     | P0.26 | Active LOW (common anode RGB)      |
| LED Green   | P0.30 | Active LOW                         |
| LED Blue    | P0.06 | Active LOW                         |

The IMU is an LSM6DS3TR-C (WHO_AM_I = 0x6A). The older LSM6DS3 returns 0x69.
Both use the same register map for accelerometer reads.

## BLE Protocol

- Service UUID: `b0b0ba90-0001-4000-8000-000000000000`
- Accel characteristic UUID: `b0b0ba90-0002-4000-8000-000000000000`
- Notification payload: 20 bytes (fits default ATT MTU of 23 - 3 overhead)

```
Byte 0:    sensor_id (0=TOP, 1=MID, 2=BOT)
Byte 1:    sequence (wrapping u8, for drop detection)
Bytes 2-7: sample 0 — x_i16_le, y_i16_le, z_i16_le
Bytes 8-13: sample 1
Bytes 14-19: sample 2
```

3 samples × 6 bytes + 2 header = 20 bytes per notification.
At 104 Hz ODR → ~35 notifications/sec per sensor.

## Key Architecture Decisions

- **nrf-softdevice** for BLE (not `trouble`/`nrf-sdc`) — battle-tested, Nordic-certified
  stack. The factory Adafruit bootloader already includes S140 v7.3.0, so no separate
  SoftDevice flash is needed. Flash firmware via UF2 drag-and-drop over USB.
- **Embassy async** runtime — same model as the Pico Embassy work, uses `embassy-nrf`
  HAL with interrupt priorities P2/P3 (SoftDevice reserves P0, P1, P4).
- **No workspace Cargo.toml** — firmware and pc-client target different architectures.
  Protocol is a path dependency from both. Don't add a root workspace.
- **Default ATT MTU (23)** — avoids MTU negotiation complexity. 3 samples/packet is
  the max that fits. If MTU negotiation is added later, increase `SAMPLES_PER_PACKET`.
- **±4g accelerometer range** — `CTRL1_XL = 0x48`. Change to `0x4C` for ±8g or `0x44`
  for ±2g. Update `AccelSample::to_g()` scale factor if range changes:
  - ±2g: 0.000061
  - ±4g: 0.000122
  - ±8g: 0.000244
  - ±16g: 0.000488

## Per-Board Configuration

Each XIAO gets a unique `SENSOR_ID` constant in `firmware/src/main.rs`:

```rust
const SENSOR_ID: u8 = 0;  // 0=BAG_TOP, 1=BAG_MID, 2=BAG_BOT
```

This is the ONLY change needed between boards. It sets the BLE advertisement
name and the sensor_id byte in every packet.

## Memory Layout (firmware)

Defined in `firmware/memory.x`. Application flash starts after the SoftDevice:

```
Flash: 0x00027000 - 0x00100000 (868 KB application)
RAM:   0x20006000 - 0x20040000 (232 KB application)
```

If the SoftDevice reports insufficient RAM at startup (via defmt/RTT log),
increase the RAM ORIGIN and decrease LENGTH by the same amount.

## Common Issues

- **Version conflicts between embassy-nrf and nrf-softdevice:** Pin both to the
  same git revision of the Embassy repo, or use compatible crates.io versions.
  The published versions may lag — git deps are standard in this ecosystem.
- **IMU reads all zeros:** The `_imu_pwr` Output on P1.08 must stay alive (not
  dropped). It's held in `main()` but watch for refactors that move it into a
  block scope.
- **SoftDevice "not enough RAM":** Bump `memory.x` RAM ORIGIN. The SD prints
  the minimum required address.
- **BLE not advertising:** Verify the UF2 flashed successfully — the XIAO USB
  drive should disappear after copy, indicating a reboot. Check INFO_UF2.TXT
  on the bootloader drive to confirm S140 v7.3.0 is present. If using SWD,
  ensure SoftDevice hex was flashed before the application.
- **PC client "permission denied":** Linux needs `CAP_NET_RAW` for BLE. Use
  setcap or run as root.

## Dependency Notes

**Note on logging:** `defmt` + `defmt-rtt` logs are only visible when flashing
via SWD with probe-rs. When flashing via UF2, the firmware runs but RTT logs
are not accessible. For debugging without a probe, consider adding USB CDC
serial output as a future enhancement.

### Firmware
- `embassy-executor`, `embassy-nrf`, `embassy-time`, `embassy-sync` — async embedded runtime
- `nrf-softdevice`, `nrf-softdevice-s140` — BLE stack wrapping Nordic S140
- `defmt` + `defmt-rtt` + `panic-probe` — logging via RTT debug channel
- `cortex-m`, `cortex-m-rt` — ARM Cortex-M startup and runtime

### PC Client
- `btleplug` — cross-platform BLE central library
- `ratatui` + `crossterm` — terminal UI
- `tokio` — async runtime
- `csv` + `chrono` — data logging
- `uuid` — BLE UUID handling

## Future Enhancements (not yet implemented)

- Impact detection algorithm (threshold + debounce on magnitude)
- Punch classification using IMU signature patterns
- Battery level monitoring via SAADC
- OTA firmware update via DFU
- MTU negotiation for higher throughput
- Gyroscope data (currently disabled to save power)
- Web dashboard alternative using WebBLE
