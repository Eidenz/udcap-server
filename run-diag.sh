#!/usr/bin/env bash
# Diagnostic runner for the UDCAP community driver core on Linux.
# Installs the udev rule (durable tty + raw-USB access) if missing, then runs
# the phase-traced diagnostic tool. All output is unbuffered and saved to a log.
#
# Usage:  ./run-diag.sh
# Stop with Ctrl-C (it works now).

set -uo pipefail
cd "$(dirname "$0")"

BIN="build/UdCapDiag"
LOG="/tmp/udcap_diag.log"
RULE_SRC="udev/70-udcap.rules"
RULE_DST="/etc/udev/rules.d/70-udcap.rules"

# 1) Ensure the diagnostic binary exists.
if [[ ! -x "$BIN" ]]; then
    echo "Building diagnostic tool..."
    (cd build && cmake --build . --target UdCapDiag -j) || {
        echo "Build failed. Configure first with: (cd build && cmake -DBUILD_TEST_TOOLS=ON ..)"
        exit 1
    }
fi

# 2) Ensure the udev rule is installed (grants tty + raw-USB access, survives replug).
if ! cmp -s "$RULE_SRC" "$RULE_DST" 2>/dev/null; then
    echo "Installing udev rule (sudo) -> $RULE_DST"
    sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=usb --subsystem-match=tty
    echo "udev rule installed. If access still fails, unplug/replug the dongle once."
    sleep 1
fi

# Kill any stale instance still holding the serial port open.
pkill -f 'UdCapDiag|UdCapCommunityDriverCoreTest' 2>/dev/null && { echo "Killed stale instance(s)."; sleep 1; }

echo
echo "=== Running UDCAP diagnostic ==="
echo "Power on the gloves and make sure they are LINKED (not in pairing/green mode)."
echo "Output is unbuffered and also saved to: $LOG"
echo
"$BIN" "$@" 2>&1 | tee "$LOG"
