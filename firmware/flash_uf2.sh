#!/usr/bin/env bash
set -euo pipefail

# ── UF2 Flash Script for XIAO nRF52840 ─────────────────────────────
#
# Usage:
#   ./flash_uf2.sh [MOUNT_POINT]
#
# Default mount point: /media/$USER/XIAO-SENSE (or /media/$USER/XIAO-BOOT)
#
# Prerequisites:
#   cargo install cargo-binutils
#   rustup component add llvm-tools
#   pip install --user adafruit-nrfutil   (for uf2conv.py, or use the bundled one)
#
# Steps:
#   1. Builds firmware in release mode
#   2. Converts ELF → raw binary via cargo-objcopy
#   3. Converts binary → UF2 with nRF52840 family ID (0xADA52840)
#   4. Copies UF2 to the mounted XIAO bootloader drive
#

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/target/thumbv7em-none-eabihf/release"
FIRMWARE_NAME="boxing-bag-firmware"
UF2_FAMILY="0xADA52840"
APP_START_ADDR="0x27000"

# ── Find mount point ───────────────────────────────────────────────
if [ -n "${1:-}" ]; then
    MOUNT="$1"
elif [ -d "/media/$USER/XIAO-SENSE" ]; then
    MOUNT="/media/$USER/XIAO-SENSE"
elif [ -d "/media/$USER/XIAO-BOOT" ]; then
    MOUNT="/media/$USER/XIAO-BOOT"
elif [ -d "/run/media/$USER/XIAO-SENSE" ]; then
    MOUNT="/run/media/$USER/XIAO-SENSE"
elif [ -d "/run/media/$USER/XIAO-BOOT" ]; then
    MOUNT="/run/media/$USER/XIAO-BOOT"
else
    echo "ERROR: XIAO not found in bootloader mode."
    echo ""
    echo "Double-tap the reset button on the XIAO — a USB drive named"
    echo "XIAO-SENSE or XIAO-BOOT should appear."
    echo ""
    echo "Or specify the mount point manually:"
    echo "  ./flash_uf2.sh /path/to/XIAO-SENSE"
    exit 1
fi

echo "==> XIAO bootloader drive found at: $MOUNT"

# ── Verify it's actually a XIAO bootloader ─────────────────────────
if [ -f "$MOUNT/INFO_UF2.TXT" ]; then
    echo "==> Bootloader info:"
    cat "$MOUNT/INFO_UF2.TXT"
    echo ""
else
    echo "WARNING: INFO_UF2.TXT not found — are you sure this is the XIAO?"
fi

# ── Build ──────────────────────────────────────────────────────────
echo "==> Building firmware (release)..."
cd "$SCRIPT_DIR"
cargo build --release

# ── Convert ELF → Intel HEX ───────────────────────────────────────
echo "==> Converting ELF → HEX..."
cargo objcopy --release -- -O ihex "$BUILD_DIR/$FIRMWARE_NAME.hex"

# ── Convert HEX → UF2 ─────────────────────────────────────────────
echo "==> Converting HEX → UF2..."

# Try to find uf2conv.py
UF2CONV=""
for candidate in \
    "$SCRIPT_DIR/tools/uf2conv.py" \
    "$(command -v uf2conv.py 2>/dev/null || true)" \
    "$HOME/.local/bin/uf2conv.py"; do
    if [ -n "$candidate" ] && [ -f "$candidate" ]; then
        UF2CONV="$candidate"
        break
    fi
done

if [ -z "$UF2CONV" ]; then
    echo ""
    echo "ERROR: uf2conv.py not found."
    echo ""
    echo "Install it with one of:"
    echo "  pip install --user adafruit-nrfutil"
    echo ""
    echo "Or download it directly:"
    echo "  mkdir -p tools && curl -o tools/uf2conv.py \\"
    echo "    https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py"
    exit 1
fi

python3 "$UF2CONV" \
    "$BUILD_DIR/$FIRMWARE_NAME.hex" \
    -c \
    -o "$BUILD_DIR/$FIRMWARE_NAME.uf2" \
    -f "$UF2_FAMILY"

echo "==> UF2 file: $BUILD_DIR/$FIRMWARE_NAME.uf2"

# ── Copy to XIAO ──────────────────────────────────────────────────
echo "==> Copying to $MOUNT ..."
cp "$BUILD_DIR/$FIRMWARE_NAME.uf2" "$MOUNT/"

# Wait a moment for the filesystem to sync
sync

echo ""
echo "==> Done! The XIAO should reboot into the new firmware."
echo "    (The USB drive will disappear — that's normal.)"
