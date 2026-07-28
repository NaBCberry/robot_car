#!/bin/sh
set -eu

echo "Python: $(python3 --version 2>&1)"
echo "Video devices (read-only):"
find /dev -maxdepth 1 -name 'video*' -print 2>/dev/null || true
echo "TTY candidates (read-only; no port is selected automatically):"
find /dev -maxdepth 1 \( -name 'ttyS*' -o -name 'ttyAMA*' -o -name 'ttyUSB*' -o -name 'ttyACM*' \) -print 2>/dev/null || true
if command -v ip >/dev/null 2>&1; then
  echo "CAN links (read-only):"
  ip -details link show type can 2>/dev/null || true
fi
