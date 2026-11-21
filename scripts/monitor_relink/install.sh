#!/bin/bash
# Install the monitor auto-relink fix on the robot (Orin). Run once:
#   sudo bash scripts/monitor_relink/install.sh
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -m 755 "$DIR/monitor_relink.sh" /usr/local/bin/home2-monitor-relink.sh
install -m 644 "$DIR/home2-monitor-relink.service" /etc/systemd/system/home2-monitor-relink.service
install -m 644 "$DIR/99-home2-monitor-relink.rules" /etc/udev/rules.d/99-home2-monitor-relink.rules

systemctl daemon-reload
systemctl enable home2-monitor-relink.service
udevadm control --reload-rules
echo "Installed. Fixing the monitor now..."
systemctl start home2-monitor-relink.service
journalctl -t home2-monitor-relink -n 5 --no-pager
