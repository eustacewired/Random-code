# FIRMWARE
Relevant hardware
- Arduino nano v3
- 4 12V DC Geared motors with rear wheels encoded (different frictions hence run at different PWMs)
- IMU (MPU6050)

## Pinout
Right Driver Output Mapping
OUT1: Connect to Right Front (RF) - terminal.
OUT2: Connect to Right Front (RF) + terminal.
OUT3: Connect to Right Back (RB) + terminal.
OUT4: Connect to Right Back (RB) - terminal.
Left Driver Output Mapping
OUT1: Connect to Left Back (LB) + terminal.
OUT2: Connect to Left Back (LB) - terminal.
OUT3: Connect to Left Front (LF) - terminal.
OUT4: Connect to Left Front (LF) + terminal.

M1= + M2= -

Arduino connections
ENA (Right Front Speed): Connect to Arduino D5 (PWM)
ENB (Right Back Speed): Connect to Arduino D6 (PWM)
IN1 + IN3 (Forward Bridged): Connect to Arduino D7
IN2 + IN4 (Reverse Bridged): Connect to Arduino D8
Left
ENA (Left Back Speed): Connect to Arduino D9 (PWM)
ENB (Left Front Speed): Connect to Arduino D10 (PWM)
IN1 + IN3 (Forward Bridged): Connect to Arduino A0 (Moved from D9)
IN2 + IN4 (Reverse Bridged): Connect to Arduino A1 (Moved from D10)

Rear Right  Motor (RB) Encoder Board:
V (VCC) / M1 → Buck 5V Pin
G (GND) / M2 → Buck GND Pin
A (Encoder A) / C1 → Arduino D2
B (Encoder B) / C2→ Arduino D4

Rear Left Motor (LB) Encoder Board:
V (VCC) → Buck 5V Pin
G (GND) → Buck GND Pin
A (Encoder A) → Arduino D12
B (Encoder B) → Arduino D3 

Stepper
A4988 DIR (Direction): Arduino A2
A4988 STEP (Pulse): Arduino A3

Servo:
Signal Wire: D11 (Controlled via the ServoTimer2 library)

IMU
A4 (SDA)
A5 (SCL)

**IMU test code for genuine-ness**
A genuine MPU-6050 returns 0x68 from WHO_AM_I (register 0x75).
Temperature: raw/340 + 36.53 should read near room temperature.
Accelerometer: z should read about 16384 counts when flat at ±2 g.
Gyro: integrate z over a slow hand-turned 360° rotation and see if you get about 360°. That checks the 131 LSB/(°/s) scale.
```
#include <Wire.h>

// MPU-6050 Default I2C Address is 0x68 (Changes to 0x69 if AD0 pin is pulled HIGH)
const uint8_t MPU_ADDR = 0x68; 
const uint8_t WHO_AM_I_REG = 0x75;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open
  
  Wire.begin();
  Wire.setClock(100000); // Standard 100kHz I2C clock

  Serial.println("\n--- MPU-6050 WHO_AM_I Test ---");

  // 1. Point to the WHO_AM_I register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I_REG);
  
  // End transmission but keep the connection active (restart)
  byte error = Wire.endTransmission(false); 

  if (error != 0) {
    Serial.print("Error communicating with I2C device. Code: ");
    Serial.println(error);
    Serial.println("Check your wiring, power (VCC/GND), and I2C pull-ups.");
    return;
  }

  // 2. Request 1 byte from the register
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t chip_id = Wire.read();
    
    Serial.print("Received WHO_AM_I Value: 0x");
    if (chip_id < 0x10) Serial.print("0"); // Print leading zero
    Serial.println(chip_id, HEX);

    // 3. Interpret the result
    if (chip_id == 0x68) {
      Serial.println(">> SUCCESS: Your chip reports 0x68. It is a genuine MPU-6050!");
    } else if (chip_id == 0x70 || chip_id == 0x72) {
      Serial.println(">> NOTICE: Reports 0x70/0x72. This is likely an MPU-6500 or clone.");
    } else if (chip_id == 0x98) {
      Serial.println(">> NOTICE: Reports 0x98. This is likely an ICM-20689 or clone.");
    } else {
      Serial.println(">> UNKNOWN: This is a clone or a completely different IMU.");
    }
  } else {
    Serial.println("Failed to read data from the chip.");
  }
}

void loop() {
  // Nothing to repeat
}

```

## Firmware files
**A. Real time layer**
1. config.h: holds every pin, calibration number and tuning constant.
2. encoder_driver.h: counts rear-wheel encoder edges in an interrupt using direct port reads. It has a fallback that counts only channel B edges, with direction taken from the drive command. The fallback costs roughly 40–50 bytes of the Nano’s 2 KB SRAM (the OpenLoopWheel object and a few flags) and a small amount of flash. The pin-change interrupt only fires on D4 edges, which you’re counting anyway. arduino-cli compile prints the exact numbers.
3. motor_driver.*:drives one side: shared direction pins, separate PWM per wheel (rear and front). configurePwmTimers() moves Timer1 to about 976 Hz so it matches Timer0. drive() applies the front trim, front minimum PWM, turn boost and a soft-start limit on PWM rise. 976 Hz on all four enables is about the best the hardware allows. D5/D6 are on Timer0, which also drives millis() and delay(). Matching Timer1 to it was the right call. The cost is audible whine and some current ripple, which doesn’t affect response.
4. pid_controller.h: From rest it holds a kick PWM, then uses feedforward on a ramped setpoint plus PI trim. It brakes before reversing and has fault detection (wrong sign, saturated, no motion).
5. imu_driver.*: raw-register MPU6050 driver with gyro bias calibration at boot.
6. serial_protocal.*: handles ASCII lines. The Pi sends v L R, o, or s, and the Nano sends READY, e ... telemetry every 50 ms, and F ... faults.

**B. ros2_control plugin**
1. arduino_comms.hpp: Uses LibSerial to parse the newest telemetry line and collects fault reports.
2. wheel.hpp: converts between ticks and radians, with a direction sign.
3. robot_system.cpp: read() integrates tick deltas using the Nano’s own clock, mirrors the rear wheels onto the fronts, and scales the IMU. write() averages front and rear commands per side and sends v, which doubles as the 500 ms watchdog heartbeat.
4. 99-auracle.rules: match each device by vendor and product ID and create stable /dev/arduino and /dev/rplidar symlinks. any USB port works, because the rules match on ID, not port. If skipped, ports have to be defined by hand

**Notes**
At 20 Hz a telemetry line is about 70 bytes, which already takes around 6 ms of a 115200 baud link. At 40 Hz it would use roughly a quarter of the link, and Serial.print blocks when the 64-byte buffer fills.
Baudrate: 500000

The fronts copy the rear PWM, and RB’s weak forward drive makes its PI demand about 2x the PWM, so RF copied that and out-ran it. That’s why RF_TRIM_FWD is 0.5 while LF is 1.0, which is a patch on a structural problem. A better design is to drive each front from the feedforward of the target speed, using the healthy side’s curve, plus a clamped share of the PI correction. Because the fronts have less friction than the rears, keep their trim slightly below 1 so they add thrust without dragging the rears.

Until RB is fixed, the 0.16 m/s forward cap (RB_FWD_MAX_TICKS_S 740) limits the whole robot, and no tuning gets past it.

# NETWORK & TIMING
## Zenoh routing
ROS2 Middleware: Zenoh
When using the official ROS 2 Jazzy Zenoh middleware (rmw_zenoh port 7447), Zenoh respects your ROS_DOMAIN_ID. It uses the domain ID to partition discovery traffic and keep separate ROS graphs isolated, exactly like standard CycloneDDS or FastDDS do. Zenoh utilizes a peer-to-peer or router-based discovery mechanism. By default rmw_zenoh does not use multicast scouting: nodes connect to a router (tcp/localhost:7447 unless configured), so the PC is pointed at the Pi's router by IP (see PC Setup). Across different networks, only one central Zenoh router is needed on a known IP (or domain name). All dynamic-IP clients simply connect to that router, and Zenoh handles the routing automatically. Zenoh maps the Domain ID to a string path inside its session expressions, meaning it can comfortably support any ID from 0 to 255 (and even higher depending on the middleware version).
Everyone else is on domain 0, so this setup uses its own unique ROS_DOMAIN_ID (89 here). It is set in three places that must match: the Pi's zenoh.service, the Pi's ~/.bashrc, and DOMAIN_ID in connect_pi.sh on the PC.

### Prerequisites
Both Pi and PC
```
# 1. Add the Zenoh Debian repository
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list

# 2. Update and install the Zenoh Router
sudo apt update
sudo apt install zenoh

# 3. Install the ROS 2 Jazzy Zenoh Middleware
sudo apt install ros-jazzy-rmw-zenoh-cpp
```
### Pi setup
A zenoh router service runs automatically on boot on the pi making it a router (/etc/systemd/system/zenoh.service)
```
[Unit]
Description=Zenoh Router Service (rmw_zenoh)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=auracle
Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp" "ROS_DOMAIN_ID=89"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && exec ros2 run rmw_zenoh_cpp rmw_zenohd"
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```
**First-time setup**
Reload the systemd daemon, enable the service to launch on boot, and start it immediately
```
sudo systemctl daemon-reload
sudo systemctl enable zenoh.service
sudo systemctl start zenoh.service
```
**Status and logs**
```
#check service status
sudo systemctl status zenoh.service
#view service logs -n 50 shows the last 50 lines, -f follows the logs in real time
sudo journalctl -u zenoh.service -n 50 -f
#restart stale service
sudo systemctl restart zenoh.service
```
For ROS nodes you run on the Pi, add this to ~/.bashrc (same domain ID as the service):
```
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=89
```
### Pc setup
To connect your PC and Raspberry Pi over a local network without internet access, they need to be on the same IP subnet and be able to find each other. Moving to a multi-network setup (e.g., across different subnets, VPNs, or cellular links) requires the PC to be explicitly pointed to the Pi's IP address. A bash script (`connect_pi.sh`) created on the PC automatically scans your local network to find the headless Pi, sets up your ROS 2 Zenoh environment (including the unique domain ID), and sets ZENOH_CONFIG_OVERRIDE so the PC connects to the Pi's router.
```
#!/bin/bash

# Configuration - Change this to your Pi's actual hostname (or pass the Pi's IP as the first argument)
PI_HOSTNAME="${1:-auracle.local}"
# Configuration - your unique ROS domain ID (must match the Pi's zenoh.service and ~/.bashrc)
DOMAIN_ID=89

echo "===================================================="
echo "📡 Initializing ROS 2 Jazzy & Zenoh LAN Connection"
echo "===================================================="

# 1. Force ROS 2 to use Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
echo "✅ Set RMW_IMPLEMENTATION to rmw_zenoh_cpp"

# 2. Set the unique ROS_DOMAIN_ID (overrides any leftover 0 in the shell)
export ROS_DOMAIN_ID=$DOMAIN_ID
echo "🆔 Active ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 3. Resolve the Pi's IP address on the local network (mDNS)
echo "🔍 Searching for your Pi ($PI_HOSTNAME)..."
PI_IP=$(getent ahostsv4 "$PI_HOSTNAME" | awk '{print $1; exit}')

# Fallback if mDNS fails: try to ping it
if [ -z "$PI_IP" ]; then
    PI_IP=$(ping -4 -c 1 "$PI_HOSTNAME" 2>/dev/null | head -n 1 | awk -F'[()]' '{print $2}')
fi

if [ -n "$PI_IP" ]; then
    echo "📌 Found Pi at IP: $PI_IP"
    
    # 4. Point rmw_zenoh at the Pi's router (rmw_zenoh does not use multicast by default)
    # (fixed: the old config file + ZENOH_ROUTER_CONFIG_URI configured the router, not the nodes)
    export ZENOH_CONFIG_OVERRIDE=$(printf 'connect/endpoints=["tcp/%s:7447"]' "$PI_IP")
    echo "🔗 ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
    echo "🚀 Ready! PC will connect to the Pi's Zenoh router at tcp://$PI_IP:7447"
else
    echo "⚠️  Could not resolve '$PI_HOSTNAME' automatically."
    echo "💡 Note: rmw_zenoh does not use multicast by default, so nodes will only look for a router on localhost."
    echo "   Retry with the Pi's IP: source ./connect_pi.sh <pi_ip>"
    unset ZENOH_CONFIG_OVERRIDE
fi

# 5. Restart the ROS 2 CLI daemon so it picks up the new RMW/config
command -v ros2 >/dev/null 2>&1 && ros2 daemon stop >/dev/null 2>&1

echo "===================================================="
```
Make the script executable, run it by sourcing it, can add it to the bashrc to avoid sourcing on every new terminal on the PC
```
chmod +x connect_pi.sh
source ./connect_pi.sh
```
## Timing
Using chrony time synchronization guarantees that both devices share identical timestamps down to the millisecond, which is crucial for ROS 2 sensor fusion and TF transforms. Chrony will use the PC's internal clock hardware crystal to keep everything synced offline.
### Prerequisites
```
# Install chrony
sudo apt update && sudo apt install -y chrony
```
### WSL setup
By default, WSL2 hides behind a NAT network, preventing your Raspberry Pi from seeing your PC's Chrony server or Zenoh traffic. Turning on Mirrored Mode mirrors your physical PC's network interface directly into WSL2. To turn on mirrored mode on WSL, press Win + R, type %userprofile%, and hit Enter. Look for a file named .wslconfig. If it doesn't exist, create it. Paste the following network configuration block:
```
[wsl2]
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true
```
Restart WSL to apply the changes in powershell as admin
```
wsl --shutdown
```
For WSL2 environments only: handle abrupt hypervisor system sleeps smoothly
```
if grep -qi microsoft /proc/version; then
    sudo bash -c "echo 'makestep 1.0 -1' >> /etc/chrony/chrony.conf"
fi
```
To allow your Raspberry Pi to communicate with your Chrony time server running inside WSL2 (Mirrored Mode), you must open UDP Port 123 on the Windows Host Firewall. Windows Defender Firewall blocks unsolicited inbound UDP traffic by default, which prevents the Pi from reaching your PC's clock. Mirrored mode is also filtered by the Hyper-V firewall that WSL uses, so a rule is needed there too. Open firewall and inject the firewall rules to allow communication (PowerShell as admin)
```
New-NetFirewallRule -DisplayName "Chrony NTP Server (WSL2 Mirrored)" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 123 -Program Any -Profile Any
# for ping requests
New-NetFirewallRule -DisplayName "Allow Inbound ICMPv4 (Ping)" -Direction Inbound -Protocol ICMPv4 -IcmpType 8 -Action Allow
# Hyper-V firewall rule for WSL (mirrored mode)
New-NetFirewallHyperVRule -Name "ChronyNTP" -DisplayName "Chrony NTP (WSL)" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 123
```
### PC setup
Run this once only (it appends to the file), then sudo systemctl restart chrony. Keep a WSL terminal open while the Pi is syncing.
```
# Configure Chrony as the network Time Server
sudo bash -c "cat <<EOF >> /etc/chrony/chrony.conf
allow all
local stratum 10
EOF"
```
`sync_time.sh` when sourced, it kills the default system tracking, starts the local Chrony server, snaps the PC clock to clear sleep/hypervisor drift, tells the Pi the PC's current IP, and sends an immediate remote trigger to force the Pi to snap its clock to yours.
```
#!/bin/bash

# Configuration - Change this to your Pi's actual hostname (or pass the Pi's IP as the first argument)
PI_HOSTNAME="${1:-auracle.local}"
PI_USER="auracle"

echo "===================================================="
echo "⏱️  Initializing Master Clock & Client Synchronization"
echo "===================================================="
echo "💻 Host Machine Identified As: $(hostname).local"

# 1. Enforce local Chrony take-over on the PC/WSL instance
echo "🧹 Optimizing Host Time Management Framework..."
if systemctl is-enabled systemd-timesyncd &>/dev/null || systemctl is-active systemd-timesyncd &>/dev/null; then
    echo "🛑 Disabling systemd-timesyncd to prevent clock conflicts..."
    sudo systemctl stop systemd-timesyncd 2>/dev/null
    sudo systemctl disable systemd-timesyncd 2>/dev/null
fi

# 2. Ensure host Chrony daemon is running cleanly
if ! systemctl is-active --quiet chrony; then
    echo "🔄 Starting Chrony server daemon..."
    sudo systemctl restart chrony
fi

# 3. Force PC clock to snap instantly (clears virtualization drift/sleep pauses)
sudo chronyc makestep
echo "✅ Local Chrony server active and tracking correctly."

# 4. Resolve the Pi's IP address on the local network (mDNS)
echo "🔍 Searching for your Pi ($PI_HOSTNAME)..."
PI_IP=$(getent ahostsv4 "$PI_HOSTNAME" | awk '{print $1; exit}')

if [ -z "$PI_IP" ]; then
    PI_IP=$(ping -4 -c 1 "$PI_HOSTNAME" 2>/dev/null | head -n 1 | awk -F'[()]' '{print $2}')
fi

if [ -n "$PI_IP" ]; then
    echo "📌 Found Pi at IP: $PI_IP"

    # The address the Pi must use to reach this PC (the Pi cannot be pointed at itself)
    PC_IP=$(ip -4 route get "$PI_IP" | awk '{for(i=1;i<NF;i++) if($i=="src"){print $(i+1); exit}}')

    # Point the Pi at this PC, then send a fast sync instruction down to the Pi
    echo "⚡ Commanding Raspberry Pi to snap its clock to PC ($PC_IP)..."
    if ssh -t -o ConnectTimeout=5 "$PI_USER@$PI_IP" "echo 'server $PC_IP iburst minpoll 2 maxpoll 4' | sudo tee /etc/chrony/sources.d/pc.sources >/dev/null && sudo chronyc reload sources >/dev/null && sudo chronyc waitsync 20 0 0 1 && sudo chronyc makestep"; then
        echo "🚀 Success! System time loops aligned across machines."
    else
        echo "❌ Pi sync failed (check the ssh login to $PI_USER@$PI_IP, then run debug_time.sh)."
    fi
else
    echo "❌ Error: Could not resolve '$PI_HOSTNAME' automatically via network."
    echo "💡 Please check physical network cables or WiFi connections, or pass the Pi's IP: source ./sync_time.sh <pi_ip>"
fi

echo "===================================================="
```
Source and activate
```
chmod +x sync_time.sh
source ./sync_time.sh
```
To confirm timing is alright, debug_time.sh looks for faults
```
#!/bin/bash

PI_HOSTNAME="${1:-auracle.local}"
PI_USER="auracle"

echo "===================================================="
echo "🩺 CHRONY PURE TIME DIAGNOSTICS SUITE"
echo "===================================================="

# 1. Check local environment context
if grep -qi microsoft /proc/version; then
    echo "ℹ️  Environment: WSL2 Virtual Machine detected."
    if ! grep -qi "networkingMode=mirrored" /mnt/c/Users/*/.wslconfig 2>/dev/null; then
        echo "⚠️  WARNING: Mirrored mode look missing in .wslconfig! Pi won't see this clock."
    fi
else
    echo "ℹ️  Environment: Native Linux Hardware."
fi

# 2. Check Local Host Chrony Health
echo -e "\n💻 Inspecting Local Master Chrony Server Status..."
if systemctl is-active --quiet chrony; then
    echo "   👉 PC Chrony service status: ACTIVE"
    echo "📊 Host tracking reference layout:"
    chronyc tracking | grep -E "Stratum|System time"
else
    echo "   ❌ FAIL: PC Chrony service is dead. Run 'sudo systemctl restart chrony'."
fi

# 3. Test Pi Network Availability & NTP Ports
echo -e "\n🛰️  Verifying Pi Client Availability..."
PI_IP=$(getent ahostsv4 "$PI_HOSTNAME" | awk '{print $1; exit}')
if [ -n "$PI_IP" ]; then
    echo "   👉 Name Resolution: Success ($PI_HOSTNAME -> $PI_IP)"
    
    if ping -4 -c 2 "$PI_HOSTNAME" &>/dev/null; then
        echo "   👉 Ping Connectivity: Success"
    else
        echo "   ❌ Ping Connectivity: Pi is not dropping packets back safely."
    fi
else
    echo "   ❌ Name Resolution: Failed to resolve '$PI_HOSTNAME'."
fi

# 4. Check Remote Chrony Sync Tracking on Pi via SSH
echo -e "\n📊 Polling Pi Remote Synchronization Metrics..."
PI_SYNC=$(ssh -o ConnectTimeout=3 "$PI_USER@${PI_IP:-$PI_HOSTNAME}" "chronyc tracking" 2>/dev/null)

if [ -n "$PI_SYNC" ]; then
    OFFSET=$(echo "$PI_SYNC" | grep "System time" | sed 's/^[^:]*: *//')
    STRATUM=$(echo "$PI_SYNC" | grep "Stratum" | awk '{print $3}')
    REF_ID=$(echo "$PI_SYNC" | grep "Reference ID" | awk '{print $4, $5}')
    
    echo "   👉 Pi Stratum Rank: $STRATUM"
    echo "   👉 Target Reference Master ID: $REF_ID"
    echo "   👉 Time Offset from Master Clock: $OFFSET"
    
    if echo "$PI_SYNC" | grep -q "Leap status     : Normal"; then
        echo "   🎉 SUCCESS: Clock synchronization loop is completely healthy!"
    else
        echo "   ⚠️  WARNING: Sync loops are tracking but not fully locked."
    fi
else
    echo "   ❌ FAIL: Could not log in via SSH to read clock registers directly."
    echo "      Windows Firewall or an offline network block might be swallowing NTP traffic."
fi

echo "===================================================="
```
### Pi setup
The Pi acts as the tracking client background service.
```
# Stop following internet sources: the PC's IP is written by sync_time.sh into /etc/chrony/sources.d/pc.sources
sudo sed -i 's/^pool/# pool/' /etc/chrony/chrony.conf
sudo sed -i 's/^server/# server/' /etc/chrony/chrony.conf
[ -f /etc/chrony/sources.d/ubuntu-ntp-pools.sources ] && sudo mv /etc/chrony/sources.d/ubuntu-ntp-pools.sources /etc/chrony/sources.d/ubuntu-ntp-pools.sources.disabled
grep -q '^sourcedir /etc/chrony/sources.d' /etc/chrony/chrony.conf || echo 'sourcedir /etc/chrony/sources.d' | sudo tee -a /etc/chrony/chrony.conf
```
Stop timesyncd and start Chrony service on boot
```
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd
sudo systemctl daemon-reload
sudo systemctl enable chrony
sudo systemctl restart chrony
```
### Launch
`ssh-copy-id auracle@auracle.local` from the PC, so the scripts can log in to the Pi. Then, on the PC (both machines on the same network, no internet needed):
```
source ./connect_pi.sh
source ./sync_time.sh
./debug_time.sh
```
If auracle.local does not resolve (common from WSL), pass the Pi's IP to each script, for example source ./connect_pi.sh 192.168.1.50.

# TELEOP + SLAM
LiDAR: RPLIDAR A1 2D 360 Degree 12 Meter Scanning Radius
## Prerequisites
Install the necessary packages for teleoperation, SLAM, and standard robot descriptions
• ros-jazzy-teleop-twist-keyboard: Allows you to control your robot's movement (linear and angular velocity) using your computer's keyboard.
• ros-jazzy-slam-toolbox: The standard, highly optimized 2D SLAM package used in ROS 2 for generating maps from laser scan data.
• ros-jazzy-navigation2: The core Nav2 architecture required to utilize the map generated by SLAM for autonomous navigation and path planning.
• ros-jazzy-nav2-bringup: Provides sample launch files, configuration scripts, and simulation setups to quickly spin up your SLAM and navigation nodes.

```
sudo apt install ros-jazzy-teleop-twist-keyboard ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup -y
```

## Command routing
1. twist_mux.yaml: Priority arbiter between cmd_vel sources: joystick/keyboard 100, tracker 50, Nav2 40, each with a 0.5 s timeout. Teleop overrides Nav2.
2. my_controllers: Base ros2_control config used by sim and real: 20 Hz update rate, the three controllers (diff drive, joint broadcaster, IMU broadcaster), joint names, default geometry. Both sim and robot load this, real robot loads my_vontrollers_real overrides. layering a base file with an override is a standard pattern. ROS 2 YAML has no conditionals, so you can’t have one file that behaves differently in sim and on the robot.
3. my_controllers_real: Real-robot overrides i.e. The calibrated geometry: wheel radius 0.033 and wheel separation 0.369, instead of 0.0239 and 0.2755, Velocity and acceleration limits: +0.16 m/s forward (RB is weak), −0.25 m/s reverse, ±1.3 rad/s, cmd_vel_timeout and the odometry covariances, and the rolling window and publish rate.

## Odom
1. ekf.yaml: robot_localization EKF. Fuses wheel odometry and the IMU into a smooth odom→base_footprint transform at 20 Hz. robot_localization ships ukf_node with the same config, but it isn’t worth it here. It computes 31 sigma points for 15 states each cycle, and a ground robot at 20 Hz is nearly linear, so you pay Pi CPU for no accuracy gain.

## Teleop controllers
1. joystick.yaml: Tuning for the controller driver i.e. speeds, smoothing time constants, acceleration caps, deadzones, button/axis mapping.
2. joy_drive.py: Turns /joy into smoothed cmd_vel_joy. Has three modes: stick, triggers and buttons.
3. joy_udp_node.py: Receives UDP packets from the Windows controller bridge and publishes /joy. Sends a neutral Joy if packets stop for 0.25 s.
4. key_drive.py: Receives UDP packets from the Windows controller bridge and publishes /joy. Sends a neutral Joy if packets stop for 0.25 s.

## SLAM
1. rplidar.launch.py: Starts the RPLidar driver on /dev/rplidar at 115200 baud, Standard scan mode, respawning if it drops.
2. mapper_params_online_async.yaml: slam_toolbox tuning parameters such as resolution, scan matching, loop closure, range limits.
3. rsp.launch.py: robot_state_publisher built from the xacro that publishes the URDF and the TF tree.
4. slam_mapping.launch.py: entry point for mapping, includes online_async_launch.py (slam_toolbox lifecycle) and optionally RViz.

**Notes**
Check ros2 run tf2_ros tf2_echo base_footprint imu_link works, and that turning left gives positive angular_velocity.z on imu_broadcaster/imu. A wrong sign means the IMU’s mounting rotation is off in the URDF.

Try predict_to_current_time: true if your build supports it, to reduce transform lag.

he A1 emits the same samples regardless of range, so a limit doesn’t reduce what the driver sends. Your consumers already crop it: slam_toolbox max_laser_range: 6.0, AMCL laser_max_range: 6.0, costmaps 2.5–3.0 m. A scan message is a small array, so the extra serialization is negligible. A laser-filter node would cost more than it saves. Some rplidar_ros builds expose a range parameter, so check ros2 param list on the node.

Wifi power save: The radio sleeps between access-point beacons (about 100 ms), and the AP holds your packets until it wakes. That saves a few hundred milliwatts and adds latency and jitter, which matches your measured 55 ms average with 380 ms spikes. It doesn’t reduce CPU or RAM, so it does nothing for a resource-constrained Pi. Turn it off: `sudo iw dev wlan0 set power_save off`

my_controllers_real.yaml:
Raise the acceleration limits (about ±1.3 linear and ±6 angular).
Set velocity_rolling_window_size to 2.
Set cmd_vel_timeout to 0.3 for a faster safe stop.
After the firmware update, set update_rate to 40.

joystick.yaml / key_drive.py: smoothing_linear 0.03, smoothing_angular 0.04, max_accel_linear 1.1, max_accel_angular 5.6. Keep publish_rate equal to the controller rate.

twist_mux.yaml: use 0.3 s timeouts to match. If your diff_cont still has use_stamped_vel, you could drop twist_stamper; check ros2 param get.

joy_udp_node.py: run it on the Pi and point the Windows bridge at the Pi’s Tailscale IP. That removes one Zenoh hop over Wi-Fi from the joystick path.

rsp.launch.py / launch_robot.launch.py: drop joint_state_publisher from 30 to 10 Hz, since it only feeds wheel TFs for RViz.

mapper_params_online_async.yaml:
If CPU or heat is a problem, go from resolution 0.03 to 0.05.
With a working IMU, retest raising distance_variance_penalty and angle_variance_penalty from 0.15 toward 0.3.

In controller_manager set update_rate: 50. That’s enough for a 25 ms Nano loop and cooler than 100 Hz. In the diff_drive_controller yaml, raise or remove the linear and angular acceleration limits. They are often the biggest source of teleop lag, and I haven’t seen that file.

Key_drive.py not really necessary since teleop_twist_keyboard works well enough

The controller loop and telemetry are 20 Hz (50 ms steps). 

Lags: The controller loop and telemetry are 20 Hz (50 ms steps). Both key_drive.py and joy_drive.py apply two cascaded low-pass filters plus an acceleration cap of 0.55 m/s², so reaching 0.16 m/s takes about 0.3 s. diff_cont adds its own 0.6 m/s² limit. The Nano’s TARGET_RAMP_TICKS_S2 = 2000 works out to about 0.42 m/s², which is slower than the Pi-side limits. It is the one actually binding. The Nano also limits PWM change per period (12 per step), the soft start (35 per step), and the kick hold (450 ms). Wi-Fi power save is still on, according to your CHANGES.md. You measured 55 ms average and 380 ms spikes, which is bigger than any of the above for remote teleop.

sudo iw dev wlan0 set power_save off, and make it permanent with the NetworkManager snippet from your CHANGES.md. In joystick.yaml and key_drive.py defaults, set smoothing_linear 0.07 → 0.03, smoothing_angular 0.09 → 0.04, max_accel_linear 1.1 and max_accel_angular 5.6. In my_controllers_real.yaml, raise diff_cont linear acceleration to ±1.3 and angular to ±6.0. Keep the two scripts just below these, as they do now. If wheels slip, back these off. 

When updating firmware Set TARGET_RAMP_TICKS_S2 to 6000, MAX_PWM_STEP to 25, KICK_MS to 300, BRAKE_MS to 120 and REVERSE_WAIT_MS to 400, Set CONTROL_PERIOD_MS to 25 and double the three fault counts so they keep the same real-time window, Set BAUDRATE to 500000 (exact on a 16 MHz AVR, and the FT232 handles it) and use BAUD_500000 in arduino_comms.hpp and the xacro. Then set controller_manager update_rate to 40 and the teleop publish_rate to match. Leave diff_cont publish_rate at 20 so the Pi’s CPU cost barely moves.

Drop joint_state_publisher from 30 Hz to 10 Hz. It only feeds wheel TFs for RViz. Run ros2_control_node with chrt -f 40 (needs rtprio rights in limits.d) and SLAM under nice -n 10, so SLAM can’t delay the control loop. Your SLAM settings (0.03 m resolution, 1° coarse search) are the biggest CPU cost. If heat is a problem, go back to 0.05 m resolution first, also check heat and load while driving with vcgencmd measure_temp and top -H -p $(pgrep ros2_control_node)

Keeping SLAM next to the sensors means only the finished map crosses Wi-Fi, a small message every couple of seconds. Nav2 needs the map on the Pi anyway.

# Navigation
## Prerequisites
Install the core Nav2 packages and bringup utilities 
```
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

## NAV2
goal in RViz → bt_navigator → global path from planner_server → controller_server follows it using the local costmap from /scan → cmd_vel_nav → twist_mux → twist_stamper → diff_cont → hardware

1. localization_nav_rviz.launch.py: Entry point. Starts localization and Nav2, optional rviz use_nav:=false runs RViz only (for the laptop).
2. navigation_launch.py: Starts Nav2’s planner_server, controller_server, behavior_server, bt_navigator and waypoint_follower. Remaps its output to cmd_vel_nav.
3. nav2_params.yaml: All Nav2 settings. Uses NavFn for global paths and Regulated Pure Pursuit for following them. Also has AMCL tuning and the local and global costmaps.
4. auracle.rviz: RViz layout with the goal tool.

## Behaviour trees
auracle_bt/ is a behavior-tree mission runner that calls NavigateThroughPoses and takes a detector label. It sits above Nav2 and isn’t needed to navigate.
=============================

# Launching
## Build

## Simulation

## Real robot
Launch robot
```Pi terminal
ros2 launch auracle_bringup launch_robot.launch.py namespace:=auracle use_joy_node:=false
```
Check if controllers active
```Pi terminal
ros2 control list_controllers -c /auracle/controller_manager
```
Teleop (teleop_twist keyboard)
```
```
Teleop (modified keyboard)
```Pi terminal
ros2 run auracle_bringup key_drive.py --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/auracle
```
SLAM (live mapping)
```PC terminal
#confirm live topics
ros2 topic list
#launch
ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle use_slam:=false
```
SLAM (headless mapping)
```
ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle use_rviz:=false
```
Save map after completion
```Pi
ros2 run nav2_map_server map_saver_cli -f ~/maps/trial_1 --ros-args -r map:=/auracle/map
```
Navigation - stop teleop and SLAM, and close rviz on the laptop. Teleop has priority 100 in twist_mux, so a running key_drive can override Nav2.
```Pi
ros2 launch auracle_bringup localization_nav_rviz.launch.py namespace:=auracle use_rviz:=false map:=$HOME/maps/trial_1.yaml
```
```PC
ros2 launch auracle_bringup localization_nav_rviz.launch.py namespace:=auracle use_nav:=false
```
Click 2D Pose Estimate and set the robot’s real position and heading. Check the particle cloud tightens around the robot and there are no TF errors. Click 2D Goal Pose on a free spot and drag to set the heading.
To confirm a goal went through, run ros2 topic echo /auracle/cmd_vel_nav --once.

**Notes**
Navigation didn't really turn my robot, and it id absolutely not accurate yet

=============================
# To-do
- Confirm all my constants are as they are defined in config.h set in three iterations
- Read messages/monitor the messages from the serial monitor comms and check if messages okay using LibSerial too -- can't coz yes the serial protocol is plain ASCII, but only one process can own /dev/arduino. If you cat it while ROS runs, the bytes get split between the two readers and corrupt both. To read either way, stop ROS first then run `picocom -b 115200 /dev/arduino`
- Can i optimise the mapping and navigation parameters using optuna or AI, I really gotta tune those parameters to match my hardware
- Add baudrate as a parameter to config.h, or where ros  parameters live, coz rpildar uses baudrate apparently
- Can i use neo_localization for something better in nav2

# To-do later
- Build on auracle dashboard maybe
- Can i make an SNN that overcomes my constraints, like corrects heading faults if clearly navigation is failing, or something, same way a human can notice a fault and take quick mitigative actions
- Gazebo sim must run on same speeds as the real robot

