#!/usr/bin/env bash
# Run the UDCAP bridge server: installs the udev rule if needed, kills any stale
# instance, then runs udcap-server (which calibrates, then publishes to shm).
#
# Usage:  ./run-server.sh            # calibrate then publish
#         ./run-server.sh --no-cal   # publish without calibration (skeleton may be invalid)

set -uo pipefail
cd "$(dirname "$0")"

BIN="build/udcap-server"
RULE_SRC="udev/70-udcap.rules"
RULE_DST="/etc/udev/rules.d/70-udcap.rules"

if [[ ! -x "$BIN" ]]; then
    echo "Building udcap-server..."
    (cd build && cmake --build . --target udcap-server -j) || {
        echo "Build failed."
        exit 1
    }
fi

if ! cmp -s "$RULE_SRC" "$RULE_DST" 2>/dev/null; then
    echo "Installing udev rule (sudo) -> $RULE_DST"
    sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=usb --subsystem-match=tty
    sleep 1
fi

pkill -f 'udcap-server|UdCapDiag' 2>/dev/null && { echo "Killed stale instance(s)."; sleep 1; }

echo "=== Starting udcap-server ==="
echo "Power on the gloves (linked, not green/pairing). Follow calibration prompts."
echo
exec "$BIN" "$@"
