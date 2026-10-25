#!/bin/bash
# One-shot PC -> Pi setup for the real robot. Run it in WSL (or native Linux) on the PC, SOURCED,
# so the ROS/Zenoh variables stay in your shell:
#
#     source pc/auracle_up.sh            # finds the Pi (cached IP first, then auracle.local)
#     source pc/auracle_up.sh 192.168.100.42
#
# It does, in order:
#   1. Finds the Pi once and caches its IP in ~/.auracle_pi_ip (fast next time, no repeated mDNS).
#   2. Exports RMW_IMPLEMENTATION, ROS_DOMAIN_ID and ZENOH_CONFIG_OVERRIDE so this PC's nodes
#      connect to the Pi's Zenoh router.
#   3. Time: makes sure chrony runs on the PC, snaps the PC clock, and (over ONE ssh login)
#      points the Pi's chrony at this PC, waits for lock and snaps the Pi clock.
#   4. Wi-Fi on the Pi: power save off now and on every boot.
#   5. The Pi prints one status block (offset, Wi-Fi, Zenoh router, domain).
# Needs: chrony on both machines, `ssh-copy-id auracle@<pi>` done FROM THIS WSL once.
# Asks for the Pi's sudo password once if needed.

PI_HOST="${1:-auracle.local}"
PI_USER="auracle"
DOMAIN_ID=89
CACHE="$HOME/.auracle_pi_ip"

_ok()  { echo "  [ok]   $*"; }
_bad() { echo "  [FAIL] $*"; }

echo "== Auracle PC setup (domain $DOMAIN_ID) =="

# --- 1. Find the Pi --------------------------------------------------------------------------
PI_IP=""
if [[ "$PI_HOST" =~ ^[0-9.]+$ ]]; then
    PI_IP="$PI_HOST"
elif [ -f "$CACHE" ] && ping -4 -c 1 -W 1 "$(cat "$CACHE")" >/dev/null 2>&1; then
    PI_IP="$(cat "$CACHE")"
else
    PI_IP=$(timeout 3 getent ahostsv4 "$PI_HOST" 2>/dev/null | awk '{print $1; exit}')
fi
if [ -z "$PI_IP" ]; then
    _bad "Pi not found. Run again with its IP: source pc/auracle_up.sh <pi_ip>"
    return 1 2>/dev/null || exit 1
fi
echo "$PI_IP" > "$CACHE"
_ok "Pi at $PI_IP"

# --- 2. ROS / Zenoh environment ---------------------------------------------------------------
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=$DOMAIN_ID
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$PI_IP:7447\"]"
_ok "RMW=zenoh, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, router tcp/$PI_IP:7447"
# The old CLI daemon keeps the previous RMW/config; stopping it is slow, so do it in the background.
( command -v ros2 >/dev/null && ros2 daemon stop >/dev/null 2>&1 ) &
disown 2>/dev/null

# --- 3a. Time: PC side -----------------------------------------------------------------------
if ! systemctl is-active --quiet chrony; then
    sudo systemctl restart chrony
fi
sudo chronyc makestep >/dev/null
PC_IP=$(ip -4 route get "$PI_IP" | awk '{for(i=1;i<NF;i++) if($i=="src"){print $(i+1); exit}}')
_ok "PC chrony running, clock snapped, PC address for the Pi: $PC_IP"

# --- 3b + 4. Pi side, one ssh login --------------------------------------------------------
REMOTE=$(cat <<'EOF'
want="server $PC_IP iburst minpoll 2 maxpoll 4"
if [ "$(cat /etc/chrony/sources.d/pc.sources 2>/dev/null)" != "$want" ]; then
    echo "$want" | sudo tee /etc/chrony/sources.d/pc.sources >/dev/null
    sudo chronyc reload sources >/dev/null
fi
sudo chronyc waitsync 15 0 0 1 >/dev/null 2>&1 || true
sudo chronyc makestep >/dev/null
sudo iw dev wlan0 set power_save off 2>/dev/null
conf=/etc/NetworkManager/conf.d/wifi-powersave-off.conf
[ -f $conf ] || printf '[connection]\nwifi.powersave = 2\n' | sudo tee $conf >/dev/null
t=$(chronyc tracking | awk -F': ' '/System time/{print $2}')
l=$(chronyc tracking | awk -F': ' '/Leap status/{print $2}')
w=$(iw dev wlan0 get power_save 2>/dev/null)
z="$(systemctl is-active zenoh-router.service) $(systemctl show zenoh-router.service -p Environment | grep -o 'ROS_DOMAIN_ID=[0-9]*')"
[ "$l" = "Normal" ] && echo "  [ok]   Pi clock synced to PC: $t" || echo "  [FAIL] Pi clock not locked yet ($l, $t). Re-run in a minute."
case "$w" in *off*) echo "  [ok]   Pi Wi-Fi $w";; *) echo "  [FAIL] Pi Wi-Fi: $w";; esac
case "$z" in active*ROS_DOMAIN_ID=$DOMAIN_ID) echo "  [ok]   Pi Zenoh router: $z";; *) echo "  [FAIL] Pi Zenoh router: $z (expected active, ROS_DOMAIN_ID=$DOMAIN_ID)";; esac
EOF
)
B64=$(printf '%s' "$REMOTE" | base64 -w0)
# Not captured: the Pi's sudo password prompt (if any) must reach your terminal.
if ! ssh -t -o ConnectTimeout=5 "$PI_USER@$PI_IP" "echo $B64 | base64 -d | PC_IP=$PC_IP DOMAIN_ID=$DOMAIN_ID bash"; then
    _bad "ssh to $PI_USER@$PI_IP failed. Run once from THIS terminal: ssh-copy-id $PI_USER@$PI_IP"
    return 1 2>/dev/null || exit 1
fi
echo "== Done. Check the link: ros2 topic list | grep auracle  (after the robot is launched) =="
