#!/usr/bin/env bash
# Live validation runner for the UDCAP community driver core on Linux.
# - Finds the UDCAP receiver dongles (WCH CH340, USB 1a86:7523)
# - Grants this user read/write on them (immediate, no re-login needed)
# - Runs the test tool, mirroring output to a log we can inspect
#
# Usage:  ./run-test.sh
# Stop with Ctrl-C when you've made the calibration gestures and seen data flow.

set -uo pipefail
cd "$(dirname "$0")"

BIN="build/UdCapCommunityDriverCoreTest"
LOG="/tmp/udcap_test.log"

if [[ ! -x "$BIN" ]]; then
    echo "ERROR: $BIN not found. Build first:"
    echo "  (cd build && cmake --build . -j)"
    exit 1
fi

# Discover CH340 serial ports (UDCAP dongles enumerate as WCH CH340 1a86:7523)
PORTS=()
for t in /dev/ttyUSB*; do
    [[ -e "$t" ]] || continue
    p="$(udevadm info -q property -n "$t" 2>/dev/null)"
    if grep -q 'ID_VENDOR_ID=1a86' <<<"$p" && grep -q 'ID_MODEL_ID=7523' <<<"$p"; then
        PORTS+=("$t")
    fi
done

if [[ ${#PORTS[@]} -eq 0 ]]; then
    echo "No CH340 (1a86:7523) serial devices found."
    echo "Plug in the UDCAP receiver dongle(s) and power on the gloves, then re-run."
    exit 1
fi

echo "Found ${#PORTS[@]} candidate receiver port(s): ${PORTS[*]}"

# Grant access for this session (prompts for sudo password once).
# This is reset if you unplug/replug; for a durable fix add yourself to the
# dialout group:  sudo usermod -aG dialout "$USER"   (then log out/in once).
needs_grant=0
for t in "${PORTS[@]}"; do
    [[ -r "$t" && -w "$t" ]] || needs_grant=1
done
if [[ $needs_grant -eq 1 ]]; then
    echo "Granting read/write on the port(s) (sudo)…"
    sudo chmod a+rw "${PORTS[@]}"
fi

echo
echo "=== Starting test tool ==="
echo "When a glove links, calibration auto-starts. Follow the prompts:"
echo "  1) 'Fist!'      -> make a fist and hold"
echo "  2) 'Adduction!' -> hold hand flat, fingers SPREAD apart"
echo "  3) 'Protract!'  -> hold hand flat, fingers TOGETHER/straight"
echo "Data (angles/quaternions/buttons) will stream after. Press Ctrl-C to stop."
echo "Full output is also saved to: $LOG"
echo
stdbuf -oL -eL "$BIN" 2>&1 | tee "$LOG"
