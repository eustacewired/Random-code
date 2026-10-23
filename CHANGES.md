# Auracle: real-robot changes log (Oct 2026)

What changed on the real robot while fixing jerky driving, curving, mapping and navigation,
what was measured, and what is still open. Start with the launch commands.

---

## 1. Launch commands

Everything robot-side runs **on the Pi** (SLAM and Nav2 included). The laptop only shows RViz.
Sending scans, transforms and drive commands over Wi-Fi caused the lag, the dropped messages and
the stop-start driving (see section 4.6).

Every terminal:
```
cd ~/auracle_ws/Random-code && source install/setup.bash
```
Every machine needs `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ROS_DOMAIN_ID=62`.

### Robot (Pi)
```
ros2 launch auracle_bringup launch_robot.launch.py namespace:=auracle use_joy_node:=false
```
Check: `diff_cont`, `joint_broad`, `imu_broadcaster` show "Configured and activated", and the lidar
prints `current scan mode: Standard`. If it prints `Cannot start scan: '80008000'` or
`operation time out`, see the lidar lockup notes in section 5.

### Teleop
Keyboard. Run it in an SSH terminal **on the Pi**, so only your keystrokes cross the Wi-Fi:
```
ros2 run auracle_bringup key_drive.py --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/auracle
```
Hold `w`/`s` (forward/back), `a`/`d` (spin), `u`/`o` and `m`/`.` (arcs). Space stops, `q`/`z` change the speed level.
The robot moves only while a key is held. A terminal repeats only the last key, so for "drive and
turn" use `u`/`o`, not `w` then `d`.

Controller paired to Windows (laptop/WSL, then Windows PowerShell):
```
ros2 run auracle_bringup joy_udp_node.py --ros-args -r __ns:=/auracle
python joy_bridge_win.py --host 100.81.100.9
```

### Mapping (SLAM on the Pi)
Pi:
```
ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle use_rviz:=false
~/auracle_tools/map_autosave.sh          # saves ~/maps/auto_latest.{yaml,pgm} every 20 s
```
Laptop (RViz only):
```
ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle use_slam:=false
```
Save a named map (Pi):
```
ros2 run nav2_map_server map_saver_cli -f ~/maps/<name> --ros-args -r map:=/auracle/map
```
Driving tips: tap the turns (wheel yaw is the weak point), drive steadily, and finish where you started.

### Navigation (Nav2 + AMCL on the Pi)
Pi (stop SLAM first):
```
ros2 launch auracle_bringup localization_nav_rviz.launch.py namespace:=auracle use_rviz:=false \
  map:=/home/auracle/auracle_ws/Random-code/src/auracle_bringup/config/map_1.yaml
```
Laptop (RViz only):
```
ros2 launch auracle_bringup localization_nav_rviz.launch.py namespace:=auracle use_nav:=false
```
Then, in RViz:
1. Use **2D Pose Estimate** to place the robot. AMCL starts at map_1's start chamber (0.40, 0.08).
2. Send goals with **2D Goal Pose**, the toolbar arrow.

Keep your hands off teleop while Nav2 drives, because teleop has priority in twist_mux.

### Firmware (flash from the Pi)
Stop ROS first, because only one program can hold the Nano's port.
```
cd ~/auracle_ws/Random-code
arduino-cli compile -b arduino:avr:nano:cpu=atmega328 --output-dir ~/auracle_tools/build firmware/auracle_firmware/auracle_firmware
~/auracle_tools/flash_nano.sh ~/auracle_tools/build/auracle_firmware.ino.hex
```

### Test and diagnostic tools (`~/auracle_tools/`, outside the repo)
| Tool | What it does |
|---|---|
| `encoder_check.py drive <pwm> <s>` / `count` | Open-loop motor test with wheels in the air, printing L/R ticks/s. `count` = turn the wheels by hand |
| `vel_check.py` | Closed-loop check (forward / stop / reverse / spin) using the same `v` commands the Pi sends |
| `raw_o.py <pwm>` | Raw Nano telemetry while driving open loop |
| `scan_calib.py record <s>` / `analyse` | Uses the lidar as reference: which way is forward in the scan, and the wheel-vs-lidar turning ratio |
| `enc_probe/`, `imu_probe/` | Standalone Nano test sketches: encoder wire edges, I2C bus scan + MPU6050 read |
| `power_log.sh` | Logs the Pi's 5 V input, core current and throttle flags every 0.5 s to `runs/power_*.log`. Starts at boot (user crontab) |
| `map_autosave.sh` | Saves the map every 20 s, alternating two slots, during mapping |
| `flash_nano.sh <hex>` | avrdude wrapper. Refuses to run while ROS holds the port |

Logs of every robot, SLAM and nav run are in `~/auracle_tools/runs/`. The latest robot log path is in
`runs/robot_latest.txt`.

---

## 2. Firmware changes (`firmware/auracle_firmware`)

| Change | Where | Why |
|---|---|---|
| Left rear encoder channel B **D13 → D12** (wire moved too) | `config.h` `PIN_LB_ENC_DIR` | D13 carries the Nano's LED and its resistor, which fights the input pull-up |
| Hand-counted ticks per wheel turn: **LB 985, RB 942** | `config.h` `LB/RB_TICKS_PER_REV` | Old 730/655 were wrong, and wrong by different amounts per side, which caused the curve (see 4.1) |
| Left/right mixing (`conditionTargets`) done in **wheel rev/s, not ticks** | `auracle_firmware.ino` | Shifting both sides by equal ticks added a yaw the Pi never asked for |
| **Arc instead of pivot**: pivot only when the inner wheel is asked to run backwards; otherwise drive the tightest possible arc | `auracle_firmware.ino` `conditionTargets` | Nav2 steering corrections used to flip arc ↔ pivot, giving aggressive back-and-forth turning |
| **Soft start**: PWM may rise ≤ 35 per 50 ms step (decreases instant); `KICK_MS` 300 → 450 | `motor_driver.cpp`, `config.h` `PWM_RISE_PER_CALL` | All four motors jumping to kick PWM at once caused a current spike that sagged the LiPo and browned out the Pi |
| Right rear counted on **channel B only** (pin-change interrupt on D4, direction from the drive command) | `encoder_driver.h`, `.ino`, `config.h` `RB_COUNT_ON_B` | Channel A (D2) dropped pulses in a probe test. Set false to return to normal quadrature once fixed |
| RB **forward** feedforward refit (`32.1 + 0.303·t`, was `9.7 + 0.1599·t`) | `config.h` `RB_FF_FWD_*` | RB lost most of its forward strength (reverse is unchanged) |
| RB forward speed cap `RB_FWD_MAX_TICKS_S` (650, then 740) | `config.h`, used in `conditionTargets` | RB reaches only ~735–1000 ticks/s forward even at PWM 255 |
| `MIN_TARGET_TICKS_S` 600 → **450** | `config.h` | Gentler starts, and leaves room for arcs under RB's forward cap |
| Per-direction front trim: **RF fwd 0.5 / rev 1.0, LF 1.0 / 1.0** (was LF 1.3, RF 1.0) | `config.h`, `motor_driver.*` | RF copied RB's inflated PWM and out-ran it, so the robot drifted left on `w`; fronts spun faster than rears |
| Rear turn boost `REAR_TURN_BOOST_PWM` 60; `FRONT_TURN_BOOST_PWM` 40 → 80; `INTEG_MAX_PWM` 60 → 100 | `config.h`, `pid_controller.h` | Skid-steer spins on the floor stalled the right side |
| **Stalls report-only** (`FAULT_STOP_ON_STALL = false`): NO_MOTION / SATURATED are logged, nothing is stopped. WRONG_SIGN still stops | `config.h`, `pid_controller.h`, `.ino` | Stopping every motor on a stall turned each stall into a stop-start "burst" and never freed the wheel |
| `FAULT_RETRY_MS` 1500 → 300 | `config.h` (changed on the PC) | Shorter pause after a latched fault |
| Open-loop fallback for the right side (`RB_ENCODER_OK`, `OpenLoopWheel`) | `pid_controller.h`, `config.h` | Drives RB on feedforward if its encoder is unusable. Currently **true** (closed loop) |

## 3. Pi-side ROS changes (`src/`)

### Hardware interface and controllers
- `ros2_control.xacro`, `robot_system.cpp`: ticks per turn **985 / 942** (were 730 / 655).
- `my_controllers_real.yaml`:
  - `wheel_radius` 0.0239 → **0.033 m**. The old value only compensated for the wrong tick counts.
  - `wheel_separation` 0.2755 → **0.369 m**, from lidar-vs-wheel turning (wheels over-report turns by ×1.14–1.51).
  - Limits: linear **+0.16 / −0.25 m/s**, angular **±1.3 rad/s**.

### Teleop
- **New `scripts/key_drive.py`**, replacing `teleop_twist_keyboard`:
  - fixed 20 Hz output with the same smoothing as `joy_drive.py`
  - hold-to-move, with repeat-gap tolerance for SSH key repeat
  - separate forward (0.16) and reverse (0.25) speeds
  - arcs at `arc_turn_scale` 0.11
- `joy_drive.py` / `joystick.yaml`: new `max_linear_reverse` (0.25); `max_linear` 0.16, `max_angular` 1.3, `wheel_separation` 0.369.

### EKF
`ekf.yaml`: **IMU disabled** (`imu0` commented out), yaw rate taken from wheel odometry
(`odom0_config` vyaw = true), because the MPU6050 isn't responding (section 5).

### SLAM (`mapper_params_online_async.yaml`)
Tuned to trust the lidar far more than wheel odometry, since there's no IMU.

| Setting | Before → after |
|---|---|
| `distance_variance_penalty` / `angle_variance_penalty` | 0.5 / 1.0 → **0.15 / 0.15** |
| `correlation_search_space_dimension` | 0.5 → **0.8** |
| `coarse_angle_resolution` | 2° → **1°** |
| `minimum_travel_distance` / `minimum_travel_heading` | 0.2 / 0.2 → **0.1 / 0.1** |
| `minimum_time_interval` | 0.5 → **0.2** |
| `min_laser_range` / `max_laser_range` | 0.4 / 12 → **0.2 / 6.0** (no self-hits seen; 0.4 dropped 6.7% of returns) |
| `resolution` | 0.05 → **0.03** |
| Loop closure | stricter (responses 0.60 / 0.70, variance 2.0), `ceres_loss_function` **HuberLoss** |
| `map_update_interval` | 5 → 2 |
| `transform_timeout` | 0.2 → 0.5 |

### Navigation (`nav2_params.yaml`)
- **AMCL** (trust the laser):
  - `alpha1–4` 0.2 → 0.6 / 0.4 / 0.3 / 0.4
  - `max_beams` 60 → 180, particles 500–2000
  - `sigma_hit` 0.1, `z_hit` 0.8 / `z_rand` 0.2
  - `update_min_d/a` 0.05
  - recovery alphas on
  - laser range 0.2–6.0 m
  - `set_initial_pose: true`
- Speeds: `desired_linear_vel` 0.13, `regulated_linear_scaling_min_speed` / `min_approach_linear_velocity` 0.10, `rotate_to_heading_angular_vel` 0.9, velocity smoother ±0.14 m/s, ±1.0 rad/s.
- RPP: **`use_rotate_to_heading: true`** (one clean turn when the heading error is > `rotate_to_heading_min_angle` 0.5 rad), **`allow_reversing: false`** (required with rotate-to-heading).

### Launch and RViz
- `slam_mapping.launch.py`: new **`use_slam`** argument (false = RViz only, for the laptop).
- `localization_nav_rviz.launch.py`: new **`use_nav`** argument (false = RViz only, for the laptop).
- `localization.launch.py`: `set_initial_pose` is **always true**. It used to be forced false on the real robot, so the global costmap timed out and Nav2 aborted its whole startup.
- `rviz/auracle.rviz`: `nav2_rviz_plugins/GoalTool` replaced by **`rviz_default_plugins/SetGoal`** on `__NS__/goal_pose`. GoalTool needs the Navigation 2 panel, which wasn't loaded, so goals went nowhere.

## 4. What was measured / found

### 4.1 Calibration
- Ticks per wheel turn, by hand (10 turns): **left 9847 → 985, right 9416 → 942**. The old values were 730 / 655, and the mismatch made the robot curve while odometry showed a straight line.
- Lidar orientation in the URDF is correct: forward in the scan is at +1°.
- Wheel turning vs lidar turning: right turns ×1.14–1.18, left turns ×1.51.

### 4.2 Right rear wheel
- **Weak going forward**, healthy going backward. Wheels in the air: +120 PWM → ~290 ticks/s, −120 → −806. Counting on A or on B gives the same numbers, and you saw it with your own eyes, so the wheel really is slower, not miscounted. Likely one half of its L298 channel, or the motor itself.
- Separately, its encoder signal sometimes drops to 0 for a while, even with continuity on D2/D4. Probe tests showed channel A losing pulses. Cause not found.

### 4.3 Turning stalls
Every stall in teleop was the right side in skid-steer spins, at speed exactly 0. That's why the turn boosts and the report-only stalls were added.

### 4.4 Power: the Pi shutting off
The Pi 5 runs from a 5 A buck converter on the 3S LiPo, through a cut USB-C cable. There's no USB-PD, so the USB ports are capped at 600 mA unless `usb_max_current_enable=1` (currently set in `/boot/firmware/config.txt`).

Observed:
- Brownout at the first bench test (motor inrush).
- Then three instant power losses during mapping and nav, one of them **while completely idle**. None were preceded by an undervoltage warning.
- Idle 5 V input: 4.90–4.99 V (a Pi 5 wants 5.1 V).

This points to a connection briefly cutting out in the power path (cut USB-C cable joints, buck terminals, LiPo leads), plus low margin.

### 4.5 IMU (MPU6050)
Telemetry is all zeros. A standalone I2C probe found the lines idle HIGH (bus not stuck), but **no device answers at any address** and the bus times out. Wiring was checked OK, so it's likely a dead module, or the Nano's A4/A5.

### 4.6 Wi-Fi
**Power saving is on** (`iw dev wlan0 get power_save`). Idle round trips to the laptop averaged 55 ms, with spikes to 380 ms. That's the cause of the RViz jumping, "queue is full", dropped teleop commands and the robot description dropping.

### 4.7 Lidar USB adapter (CP2102)
It sometimes locks up when the lidar node starts (kernel: `cp210x ttyUSB1: failed set request 0x12 status: -110`).

### 4.8 Pi housekeeping
- About 3.4 GB freed: old VS Code servers, old ROS logs, stale build folders.
- `arduino-cli` installed in `~/.local/bin`, so the Nano is compiled and flashed on the Pi.
- Nav2 installed: `ros-jazzy-navigation2`.

## 5. Pending / open issues

| Issue | Status / next step |
|---|---|
| **IMU not responding** | Unplug only the IMU's SDA/SCL and rerun `imu_probe`. "Nothing answered" without TIMEOUT = bad module, replace it. TIMEOUT still = Nano A4/A5 side. When fixed: re-enable `imu0` in `ekf.yaml` and set `odom0_config` vyaw back to false |
| **Pi power cuts** | Check and resolder the cut USB-C cable and the buck terminals. Raise the buck output to ~5.15 V at the Pi. Capacitors on the buck input. Or run the Pi from a USB-C power bank. `power_log.sh` records the 5 V history for the next cut. Consider removing `usb_max_current_enable=1` |
| **Wi-Fi power save** | `sudo iw dev wlan0 set power_save off`. To make it permanent: `printf '[connection]\nwifi.powersave = 2\n' \| sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf` |
| **Right rear weak forward** | Hardware: check its L298 channel and motor, or swap to MOSFET drivers. When repaired: raise `RB_FWD_MAX_TICKS_S`, restore `RB_FF_FWD_*`, retune `RF_TRIM_FWD`, then raise the forward speed limits again |
| **Right encoder dropouts** | Find the intermittent connection. When fixed: `RB_COUNT_ON_B = false` |
| **Lidar CP2102 lockups** | Replug the lidar, then kill the hung `rplidar_composition` so the launch respawns it. Try a black USB 2.0 port, or a powered hub. Replace the adapter if it continues |
| **Minimum-speed (hold) test on the floor** | Find the lowest PWM both rears hold smoothly, then lower `MIN_TARGET_TICKS_S` below 450 for gentler starts |
| **Front trims** | `RF_TRIM_FWD` 0.5 / `LF_TRIM` 1.0 are estimates. Straight-line test: drifts left → lower `RF_TRIM_FWD`, drifts right → raise it |
| **Odometry check** | Drive 1 m and spin 360° to confirm `wheel_radius` 0.033 and `wheel_separation` 0.369 |
| **A good real-arena map** | `map_1` is from the simulator, so it only localizes if the real field matches. Remap the real arena once power and Wi-Fi are fixed |
| **Nav tuning after the arc/pivot change** | Watch corner behaviour in tight corridors. If it turns too wide, lower `rotate_to_heading_min_angle` |
| **Uncommitted work** | Commit the latest firmware (arc instead of pivot) and `nav2_params.yaml` (rotate-to-heading). This file too |
