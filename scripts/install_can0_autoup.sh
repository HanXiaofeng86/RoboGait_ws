#!/bin/bash
# Install robust can0 auto-up (boot + USB replug).
# Run: bash scripts/install_can0_autoup.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Re-run with sudo: sudo bash $0" >&2
  exit 1
fi

cp -a /etc/systemd/system/can.service "/etc/systemd/system/can.service.bak.$(date +%Y%m%d%H%M%S)" 2>/dev/null || true
cp -a /usr/sbin/enablecan "/usr/sbin/enablecan.bak.$(date +%Y%m%d%H%M%S)" 2>/dev/null || true

cat >/usr/sbin/enablecan <<'EOF'
#!/bin/bash
# Bring SocketCAN can0 up at 500 kbit/s (idempotent).
set -e
for _ in 1 2 3 4 5 6 7 8 9 10; do
  if ip link show can0 &>/dev/null; then
    break
  fi
  sleep 0.5
done
if ! ip link show can0 &>/dev/null; then
  echo "enablecan: can0 not present yet" >&2
  exit 1
fi
ip link set can0 down 2>/dev/null || true
ip link set can0 type can bitrate 500000
ip link set can0 up
EOF
chmod 755 /usr/sbin/enablecan

cat >/etc/systemd/system/can.service <<'EOF'
[Unit]
Description=Bring up SocketCAN interface can0
After=network-pre.target systemd-udevd.service
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/enablecan
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
EOF

cat >/etc/udev/rules.d/99-can0-up.rules <<'EOF'
# USB-CAN (gs_usb): whenever can0 appears, configure and bring it up
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can0", RUN+="/usr/sbin/enablecan"
EOF

systemctl daemon-reload
udevadm control --reload-rules
systemctl enable can.service
systemctl restart can.service
udevadm trigger --action=add --subsystem-match=net --attr-match=interface=can0 2>/dev/null || true

echo "----"
ip link show can0
systemctl status can.service --no-pager -l | head -20
echo "OK: can0 should stay UP on boot and after USB-CAN replug."
