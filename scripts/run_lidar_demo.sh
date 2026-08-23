#!/usr/bin/env bash
# ~/mech-v04/scripts/run_lidar_demo.sh
#
# Automates the "start motor via C++ SDK, then run mech" sequence.
# Usage:
#   ./run_lidar_demo.sh              # default port /dev/ttyUSB0
#   ./run_lidar_demo.sh /dev/rplidar # use symlink if you set it up
#
# What it does:
#   1. Kills any leftover ultra_simple or mech process on the port
#   2. Starts the RPLIDAR motor via the Slamtec C++ SDK (short-lived)
#   3. Waits for the motor to reach speed
#   4. Launches `mech run` from the lidar-ekf example dir
#   5. On Ctrl+C, tries to cleanly stop the motor

set -u

PORT="${1:-/dev/ttyUSB0}"
BAUD=115200
SDK="$HOME/rplidar_sdk/output/Linux/Release/ultra_simple"
DEMO_DIR="$HOME/mech-v04/examples/lidar-ekf"
MECH="$HOME/mech-v04/target/release/mech"

echo "== LiDAR demo: port=$PORT baud=$BAUD =="

# --- Sanity checks ---
if [ ! -e "$PORT" ]; then
  echo "ERROR: $PORT does not exist. Is the LiDAR plugged in?"
  echo "Available serial ports:"; ls /dev/ttyUSB* 2>/dev/null || echo "  (none)"
  exit 1
fi
if [ ! -x "$SDK" ]; then
  echo "ERROR: Slamtec SDK not found at $SDK"
  echo "Build it first:  cd ~/rplidar_sdk && make"
  exit 1
fi
if [ ! -x "$MECH" ]; then
  echo "ERROR: mech binary not found at $MECH"
  echo "Build it first:  cd ~/mech-v04 && cargo build --release --features lidar_host_native"
  exit 1
fi

# --- Kill anything else holding the port ---
if command -v fuser >/dev/null 2>&1; then
  fuser -k "$PORT" 2>/dev/null && sleep 1 || true
fi

# --- Start the motor via C++ SDK, then stop (motor keeps spinning) ---
echo "[1/3] Starting motor via Slamtec SDK..."
timeout 2 "$SDK" --channel --serial "$PORT" $BAUD >/dev/null 2>&1 || true
sleep 1
echo "      Motor should be spinning now (listen for the whine)."

# --- Update the .mcfg port on the fly if user overrode it ---
if [ "$PORT" != "/dev/ttyUSB0" ]; then
  echo "[2/3] Updating mech.mcfg port to $PORT..."
  sed -i.bak "s|port: \"/dev/[^\"]*\"|port: \"$PORT\"|" "$DEMO_DIR/mech.mcfg"
  echo "      (backup saved as mech.mcfg.bak)"
fi

# --- Run mech ---
echo "[3/3] Launching mech run..."
echo "      Press Ctrl+C to stop. Motor will keep spinning until USB unplug."
echo "-----"
cd "$DEMO_DIR" && "$MECH" run

# --- Post-run info ---
echo "-----"
echo "Done. To stop the motor completely, unplug and replug the LiDAR USB."
