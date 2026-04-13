# Boxing Bag Sensor System

Stream IMU data (accelerometer + gyroscope) from 2× XIAO nRF52840 Sense boards
attached to a boxing bag, displayed in a live terminal dashboard with CSV logging.

## Architecture

```
┌───────────────┐  BLE notify   ┌──────────────────────────────┐
│  BAG_HI  (0)  │──────────────▶│                              │
│  XIAO Sense   │               │   PC Client (Rust)           │
├───────────────┤  BLE notify   │                              │
│  BAG_LO  (1)  │──────────────▶│   • btleplug BLE central     │
│  XIAO Sense   │               │   • ratatui live dashboard   │
└───────────────┘               │   • CSV data logging         │
                                └──────────────────────────────┘
```

**Data flow:** Each sensor reads the onboard LSM6DS3TR-C IMU at 104 Hz
(accel ±4g + gyro ±250 dps), packs one sample into a 14-byte BLE notification
(padded to 20 bytes), and streams to the PC client over BLE.

## Hardware

- 2× [Seeed XIAO nRF52840 Sense](https://wiki.seeedstudio.com/XIAO_BLE/)
- 1× SWD debug probe (ST-Link V2 or similar) + expansion board or pogo pins
- 2× LiPo battery (optional — can run on USB power)

### XIAO BLE Sense IMU Wiring (internal — no external wiring needed)

| Signal         | nRF52840 Pin | Notes                                  |
|----------------|-------------|----------------------------------------|
| IMU I2C SDA    | P0.07       | Internal to board                      |
| IMU I2C SCL    | P0.27       | Internal to board                      |
| IMU Power      | P1.08       | Drive HIGH with HighDrive to power IMU |
| IMU INT1       | P0.11       | Data-ready interrupt                   |
| IMU I2C Addr   | 0x6A        | LSM6DS3TR-C default                    |
| Charge Current | P0.13       | HIGH = 50 mA, LOW = 100 mA            |
| Battery ADC EN | P0.14       | LOW = enable voltage divider           |
| Battery ADC    | P0.31       | Reads Vbat/2 via voltage divider       |

## Prerequisites

```bash
# Rust toolchain
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools

# cargo-binutils for ELF → HEX conversion
cargo install cargo-binutils

# probe-rs for SWD flashing + RTT logs
cargo install probe-rs-tools

# uf2conv.py for HEX → UF2 conversion (alternative to SWD)
mkdir -p firmware/tools
curl -o firmware/tools/uf2conv.py \
  https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py
```

> **No SoftDevice download needed!** The XIAO ships from the factory with the
> Adafruit nRF52 Bootloader which already includes SoftDevice S140 v7.3.0.

### Linux BLE permissions (for PC client)

```bash
# Grant BLE capabilities (preferred)
sudo setcap 'cap_net_raw,cap_net_admin+eip' target/release/boxing-bag-client

# Or run as root
sudo cargo run --release
```

## Flashing Firmware

### Option A: SWD (recommended — gives RTT debug logs)

```bash
cd firmware
cargo run --release              # builds, flashes, shows defmt RTT logs
```

Requires a debug probe (e.g. ST-Link V2) connected to the XIAO's SWD pads.

### Option B: UF2 (no debug probe needed)

1. Set `SENSOR_ID` in `firmware/src/main.rs`:
   ```rust
   const SENSOR_ID: u8 = 0;  // 0 = HI, 1 = LO
   ```

2. Double-tap the reset button on the XIAO (a USB drive appears)

3. Flash:
   ```bash
   cd firmware
   ./flash_uf2.sh
   ```

Repeat for each board, changing `SENSOR_ID` to 0 and 1.

## Running the PC Client

```bash
cd pc-client
cargo run --release
```

### CLI options

```
boxing-bag-client [OPTIONS]

Options:
  --csv <PATH>    CSV output file (default: boxing_bag_data.csv)
  --no-tui        Headless mode, CSV-only, no terminal dashboard
```

### Dashboard controls

| Key   | Action              |
|-------|---------------------|
| `q`   | Quit                |
| `r`   | Reset peak readings |

## CSV Output Format

```csv
timestamp,device_ms,sensor_id,sensor_name,seq,ax_raw,ay_raw,az_raw,ax_g,ay_g,az_g,accel_mag_g,gx_raw,gy_raw,gz_raw,gx_dps,gy_dps,gz_dps
2026-04-13 12:53:49.365,9.0,0,BAG_HI,187,-997,616,8239,-0.1216,0.0752,1.0052,1.0153,147,-342,-27,1.29,-2.99,-0.24
```

## BLE Protocol

| Field       | Bits/Bytes | Description                                    |
|-------------|------------|------------------------------------------------|
| sequence    | 8 bits     | Wrapping counter for drop detection            |
| sensor_id   | 2 bits     | 0 = HI, 1 = LO                                |
| time_delta  | 6 bits     | ms since previous packet (0-63, saturating)    |
| imu_sample  | 12 bytes   | accel xyz (i16 LE) + gyro xyz (i16 LE)         |
| padding     | 6 bytes    | Zeros (to fill 20-byte characteristic)         |
| **Total**   | **20**     | Fits default BLE ATT MTU                       |

Custom BLE service UUID: `b0b0ba90-0001-4000-8000-000000000000`
IMU characteristic UUID: `b0b0ba90-0002-4000-8000-000000000000`

## Mounting on the Bag

Suggested placement:
- **BAG_HI (0):** Upper half, near typical punch height
- **BAG_LO (1):** Lower half, for body shots / kicks

Use adhesive velcro strips or 3D-printed enclosures. Each XIAO can be powered
by a small LiPo battery connected to the BAT pads on the board. Charging is
limited to 50 mA via firmware (P0.13).

## Project Structure

```
nrfsense-box/
├── README.md
├── CLAUDE.md
├── firmware/               # Embassy-nrf + nrf-softdevice
│   ├── Cargo.toml
│   ├── .cargo/config.toml  # target + SWD runner config
│   ├── build.rs
│   ├── memory.x            # Flash/RAM layout with SoftDevice
│   ├── flash_uf2.sh        # Build + convert + flash via USB
│   └── src/
│       ├── main.rs          # BLE peripheral + IMU streaming
│       ├── lsm6ds3.rs       # IMU driver (accel + gyro)
│       └── bin/
│           ├── ble_fake.rs  # Test: fake data over BLE
│           └── i2c_test.rs  # Test: I2C diagnostics + USB serial
├── pc-client/              # btleplug + ratatui
│   ├── Cargo.toml
│   └── src/
│       └── main.rs          # BLE central + dashboard + CSV
└── protocol/               # Shared types (no_std)
    ├── Cargo.toml
    └── src/
        └── lib.rs           # ImuPacket, UUIDs, constants
```

## Troubleshooting

**"Not enough RAM" error from SoftDevice:**
Increase the RAM origin in `firmware/memory.x`. When flashing via SWD,
the SoftDevice prints the minimum required address via defmt.

**No sensors found by PC client:**
1. Verify sensors are advertising with the nRF Connect app (iOS/Android)
2. Check BLE permissions on Linux (`setcap` or run as root)
3. Try `bluetoothctl remove <addr>` to clear stale GATT cache
4. Try `sudo systemctl restart bluetooth`

**IMU reads all zeros:**
P1.08 must use `OutputDrive::HighDrive` — the GPIO directly powers the IMU.
Standard drive (~2 mA) isn't enough. Also ensure the `_imu_pwr` variable
is not dropped.

**White blinking LEDs:**
Firmware panicked. Flash via SWD (`cargo run --release`) to see defmt logs.

**Battery monitor shutting down on USB power:**
Normal — the ADC reads noise (~600 mV) with no battery. The firmware
ignores readings below 1V to avoid false shutdowns.
