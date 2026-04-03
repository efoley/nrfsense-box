# Boxing Bag Sensor System

Stream accelerometer data from 3× XIAO nRF52840 Sense boards attached to a
boxing bag, displayed in a live terminal dashboard with CSV logging.

## Architecture

```
┌───────────────┐  BLE notify   ┌──────────────────────────────┐
│  BAG_TOP (0)  │──────────────▶│                              │
│  XIAO Sense   │               │   PC Client (Rust)           │
├───────────────┤  BLE notify   │                              │
│  BAG_MID (1)  │──────────────▶│   • btleplug BLE central     │
│  XIAO Sense   │               │   • ratatui live dashboard   │
├───────────────┤  BLE notify   │   • CSV data logging         │
│  BAG_BOT (2)  │──────────────▶│                              │
│  XIAO Sense   │               └──────────────────────────────┘
└───────────────┘
```

**Data flow:** Each sensor reads the onboard LSM6DS3TR-C IMU at 104 Hz, batches
3 samples into a 20-byte BLE notification (~35 notifications/sec), and streams
to the PC client over BLE.

## Hardware

- 3× [Seeed XIAO nRF52840 Sense](https://wiki.seeedstudio.com/XIAO_BLE/)
- 1× SWD debug probe (J-Link, CMSIS-DAP, or a DAPLink board)
- Soldering iron + fine wire for SWD pads on the back of each XIAO

### XIAO BLE Sense IMU Wiring (internal — no external wiring needed)

| Signal         | nRF52840 Pin | Notes                        |
|----------------|-------------|-------------------------------|
| IMU I2C SDA    | P0.07       | Internal to board             |
| IMU I2C SCL    | P0.27       | Internal to board             |
| IMU Power EN   | P1.08       | Drive HIGH to power IMU       |
| IMU INT1       | P0.11       | Data-ready interrupt          |
| IMU I2C Addr   | 0x6A        | LSM6DS3TR-C default           |

### SWD Debug Pads

The SWD pads are on the **back** of the XIAO board — tiny pads labeled
`SWDIO`, `SWCLK`, `GND`, and `3V3`. You'll need to solder thin wires or use
pogo pins to connect your debug probe.

## Prerequisites

```bash
# Rust toolchain
rustup target add thumbv7em-none-eabihf
rustup component add llvm-tools

# cargo-binutils for ELF → HEX conversion
cargo install cargo-binutils

# uf2conv.py for HEX → UF2 conversion
mkdir -p firmware/tools
curl -o firmware/tools/uf2conv.py \
  https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py
```

> **No SoftDevice download needed!** The XIAO ships from the factory with the
> Adafruit nRF52 Bootloader which already includes SoftDevice S140 v7.3.0.

### Linux BLE permissions (for PC client)

```bash
# Option A: run as root (not ideal)
sudo cargo run --release

# Option B: grant BLE capabilities (preferred)
sudo setcap 'cap_net_raw,cap_net_admin+eip' target/release/boxing-bag-client
```

## Flashing Firmware (USB — no debug probe needed)

The XIAO's built-in UF2 bootloader lets you flash over USB by drag-and-drop.

### Step 1: Set sensor ID

Edit `SENSOR_ID` in `firmware/src/main.rs` for each board:

```rust
// firmware/src/main.rs, line ~30
const SENSOR_ID: u8 = 0;  // 0 = TOP, 1 = MID, 2 = BOT
```

### Step 2: Enter bootloader mode

Double-tap the reset button on the XIAO. A USB mass storage drive named
`XIAO-SENSE` (or `XIAO-BOOT`) will appear on your PC.

### Step 3: Flash

```bash
cd firmware
chmod +x flash_uf2.sh
./flash_uf2.sh
```

The script builds the firmware, converts to UF2, and copies it to the XIAO.
The board reboots automatically and starts advertising over BLE.

Repeat for each board, changing `SENSOR_ID` to 0, 1, and 2.

### Alternative: SWD flashing (if you have a debug probe)

If you want RTT debug logging via `defmt`, you'll need SWD. Uncomment the
`runner` line in `firmware/.cargo/config.toml` and use `cargo run --release`.
The Seeed expansion board ($17) or soldering to the SWD pads on the back of
the XIAO both work. An ST-Link V2 clone is supported by probe-rs.

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
timestamp,sensor_id,sensor_name,seq,x_raw,y_raw,z_raw,x_g,y_g,z_g,magnitude_g
2026-04-02 10:30:15.123,0,BAG_TOP,42,-1234,5678,-9012,-0.1505,0.6927,-1.0995,1.3148
```

## BLE Protocol

| Field       | Bytes | Description                                |
|-------------|-------|--------------------------------------------|
| sensor_id   | 1     | 0 = TOP, 1 = MID, 2 = BOT                 |
| sequence    | 1     | Wrapping counter for drop detection        |
| samples[3]  | 18    | 3× (x_i16_le, y_i16_le, z_i16_le)         |
| **Total**   | **20**| Fits default BLE ATT MTU                   |

Custom BLE service UUID: `b0b0ba90-0001-4000-8000-000000000000`
Accel characteristic UUID: `b0b0ba90-0002-4000-8000-000000000000`

## Mounting on the Bag

Suggested placement:
- **BAG_TOP (0):** Upper third, near the top of the bag
- **BAG_MID (1):** Center, at typical punch height
- **BAG_BOT (2):** Lower third, for body shots / kicks

Use adhesive velcro strips or 3D-printed enclosures. Each XIAO can be powered
by a small LiPo battery connected to the BAT pads on the board.

## Project Structure

```
boxing-bag-sensors/
├── README.md
├── firmware/               # Embassy-nrf + nrf-softdevice
│   ├── Cargo.toml
│   ├── .cargo/config.toml  # target config
│   ├── build.rs
│   ├── memory.x            # Flash/RAM layout with SoftDevice
│   ├── flash_uf2.sh        # Build + convert + flash via USB
│   └── src/
│       ├── main.rs          # BLE peripheral + streaming loop
│       └── lsm6ds3.rs       # Minimal IMU driver
├── pc-client/              # btleplug + ratatui
│   ├── Cargo.toml
│   └── src/
│       └── main.rs          # BLE central + dashboard + CSV
└── protocol/               # Shared types (no_std)
    ├── Cargo.toml
    └── src/
        └── lib.rs           # AccelPacket, UUIDs, constants
```

## Troubleshooting

**Verify your bootloader first:**
Double-tap reset, open the `INFO_UF2.TXT` file on the mounted drive, and
confirm it says `SoftDevice: S140 7.3.0` (or similar v7.x).

**"Not enough RAM" error from SoftDevice:**
Increase the RAM origin in `firmware/memory.x`. When flashing via SWD,
the SoftDevice prints the minimum required address via defmt. With UF2
flashing you won't see this log — try bumping ORIGIN to `0x20008000` and
LENGTH to `224K` as a first step.

**No sensors found by PC client:**
1. Verify sensors are advertising with the nRF Connect app (iOS/Android)
2. Check BLE permissions on Linux (`setcap` or run as root)
3. Make sure you're within BLE range (~10m)

**IMU reads all zeros:**
Check that P1.08 (IMU power enable) is being driven HIGH. The `_imu_pwr`
variable must stay alive — don't let it drop.

**High packet drop rate:**
- Reduce distance between sensors and PC
- Reduce `event_length` in SoftDevice config if other BLE traffic interferes
- Consider requesting a shorter connection interval
