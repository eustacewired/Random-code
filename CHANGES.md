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

## 4.9 Findings 2026-10-23 (afternoon)
- **IMU: the I2C clock line SCL (A5) cannot be pulled LOW**: the Nano drives it LOW and it still reads HIGH (`~/auracle_tools/i2c_pins`). SDA (A4) is fine. With SCL stuck HIGH no I2C device can answer, which is why a brand-new MPU6050 also reads zero. Either something on the SCL net is tied to 5 V (wire touching 5 V, wrong module pin, solder bridge) or the Nano's A5 pin is damaged.
- Pi clock not synchronised (`chronyc tracking`: Not synchronised, no `/etc/chrony/sources.d/pc.sources`), and logs show the clock jumped. No `zenoh.service` on the Pi, and the Pi uses `ROS_DOMAIN_ID=62` (README_A says 42).
- New firmware test commands: `p <LB> <LF>` / `q <RB> <RF>` (raw signed PWM per wheel) and `g <kp> <ki>` / `h <kp> <ki>` (live left/right PI gains). Normal driving is unchanged.
- New tools: `~/auracle_tools/front_offset_test.py` (interactive: press ENTER when each wheel starts/stops) and `~/auracle_tools/pid_tune.py` (automatic step tests, `--sweep` ranks a kp x ki grid).

## 4.10 Findings and suggestions 2026-10-25 (network and time)
- **IMU: the Nano's A5 pin is damaged.** With the SCL wire unplugged, A5 still cannot be pulled LOW, so nothing outside the Nano is causing it. Options: replace the Nano (cleanest), or run the IMU on software I2C on two other pins (A2/A3 if the stepper isn't used yet), which needs a small `imu_driver` change.
- **Time sync works**: the Pi's chrony is locked to the PC (192.168.100.250, stratum 11, offset < 1 ms, Leap status Normal). `systemd-timesyncd` doesn't exist on this Ubuntu image (only a `timesync1` alias), so README_A's "stop timesyncd" step does nothing here and can be skipped.
- **Zenoh**: the router service on the Pi is `zenoh-router.service` (active, `ROS_DOMAIN_ID=89`, port 7447), not `zenoh.service` as README_A names it. Domain is 89 everywhere now (README_A updated, Pi `~/.bashrc`, service).
- **New `pc/auracle_up.sh`** (run in WSL, sourced) replaces `connect_pi.sh` + `sync_time.sh`, and also turns off Pi Wi-Fi power save (now and on every boot). See below.

### Suggested edits to README_A's PC scripts (if you keep them instead of `pc/auracle_up.sh`)
| Problem seen | Cause | Fix |
|---|---|---|
| `connect_pi.sh` hangs after "Ready!" | `ros2 daemon stop` at the end is slow | Run it in the background: `( ros2 daemon stop >/dev/null 2>&1 ) &` |
| Both scripts resolve `auracle.local` again | mDNS lookups from WSL are slow and sometimes time out | Resolve once, cache the IP (e.g. `~/.auracle_pi_ip`), try the cached IP with `ping -c1 -W1` first, and wrap `getent` in `timeout 3` |
| `sync_time.sh` rewrites `pc.sources` every time | Unconditional `tee` + `chronyc reload sources` | Only write and reload when the IP changed |
| "stop/disable systemd-timesyncd" | Unit doesn't exist on the Pi | Guard it: `systemctl list-unit-files \| grep -q '^systemd-timesyncd' && ...` |
| README_A says `zenoh.service` | The Pi's unit is `zenoh-router.service` | Rename in README_A, or rename the unit |

`pc/auracle_up.sh` already does all of the above, plus Wi-Fi power save, in one SSH login.

### SSH from the PC
Run the scripts **in WSL** (where chrony serves the time and the ROS nodes run). The script makes its own SSH connection to the Pi, so the SSH key must belong to WSL: run `ssh-copy-id auracle@<pi_ip>` once **from a WSL terminal** (Windows PowerShell has a separate `~/.ssh`). For your own interactive sessions, any `ssh auracle@auracle.local` works, from WSL or PowerShell.

## 4.11 Changes 2026-10-25 (timing, baud, IMU pins)
- **500000 baud and a 40 Hz loop**: `BAUDRATE` 500000 (`arduino_comms.hpp` `BAUD_500000`), `CONTROL_PERIOD_MS` 50 -> 25, controller_manager `update_rate` 40, joystick / key_drive `publish_rate` 40. Re-timed so behaviour per second is unchanged or quicker: `PWM_RISE_PER_CALL` 35 -> 18, `SPEED_FILTER_ALPHA` 0.4 -> 0.3, `KICK_MS` 450 -> 350, `BRAKE_MS` 200 -> 120, `REVERSE_WAIT_MS` 800 -> 500, fault counts doubled (12 / 80 / 40) so their time windows stay the same, `TARGET_RAMP_TICKS_S2` 2000 -> 4000. Verified: 40.2 Hz telemetry, 24-26 ms spacing, 0 garbled lines. The test tools in `~/auracle_tools` use 500000 too (`flash_nano.sh` keeps 115200: that is the bootloader's upload speed).
- **IMU on software I2C**: `imu_driver.cpp` no longer uses `Wire`. It bit-bangs open-drain I2C on `PIN_IMU_SCL` A2 / `PIN_IMU_SDA` A3 (~100 kHz, ~1.5 ms per read, no wait loops), with a 9-clock bus recovery at boot and `whoAmI()`.
- **IMU is switched OFF (`IMU_ENABLED = false`)** until the A2/A3 wiring is checked. A pin probe found A3 held hard at 5 V and A2 at ground, which is what the IMU's VCC and GND pins on A3/A2 would look like. Driving a line wired to VCC LOW shorts it and can kill the pin (possibly how A5 died). A5 re-tested with nothing attached: still cannot be pulled LOW, so it is damaged.

## 4.12 Nano pin scan 2026-10-25
Every pin that's safe to drive was driven LOW/HIGH and read back with nothing attached (`~/auracle_tools/pin_scan`):
A0, A1, A4, D7, D8, D11 ok. **A3 and A5 stuck HIGH (cannot be driven LOW): damaged.** A2 drives fine but its pull-up reads 0 (something sinks it to ground): unreliable. D13 pull-up reads 0 because of the onboard LED (fine as an output only). Three of six analog pins are faulty, so the Nano is partly damaged and worth replacing.
IMU moved to software I2C on **SDA = A4, SCL = D11** (`PIN_IMU_SDA` / `PIN_IMU_SCL`). D11 was reserved for the servo in README_A; the servo only needs an output and can use A2.

## 4.13 Calibration session 2026-10-25 (steps 0-8, wheels in the air)
| Step | Result | Applied |
|---|---|---|
| 0 IMU pins | Pin scan: A3, A5 stuck HIGH; A2 pull-up reads 0. IMU on **SDA A4 / SCL D11** answers at 0x68, WHO_AM_I 0x72 | `PIN_IMU_SDA` A4, `PIN_IMU_SCL` 11, `IMU_ENABLED` true |
| 1 IMU at rest | 40.1 Hz with IMU reads; gravity 1.06 g (clone scale), tilt 3.2 deg, gyro z bias 0.004 deg/s (0.2 deg/min drift), noise 0.07 deg/s | none needed |
| 2 IMU axes | Hand turns: left +354.6 deg, right -352.6 deg (for 360) | sign correct, no URDF change |
| 3 Encoders | LB 987/turn (A+B, counts down backward). RB **990**/turn with A+B (9880/10 fwd, -2002/2 back); B-only gave 992. The old 942 was A dropping pulses | `RB_TICKS_PER_REV` / xacro 990; `RB_COUNT_ON_B` false |
| 4 Directions | All four turn forward. LF needs more than PWM 150 to start; it hums first (damaged, high friction). RB forward now ~= reverse (913 vs -938 at 150): the "weak forward" was the wiring fault | RB caps removed |
| 5 Start/stop PWM | LF 96/54 fwd, 214/132 rev; RF 56/32, 58/28; LB 56/30, 60/34; RB 56/26, 48/30 | new front model: `front = rear + OFFSET`, `MIN`, front `KICK` (`LF/RF_OFFSET_*`, `*_KICK_FRONT_*`, `LF/RF_MIN_*`); rear kicks LB 64/69, RB 64/55 |
| 6 Rear power model | Both rears nearly identical; ~1420 ticks/s at PWM 250 (~0.34 m/s in air) | `LB_FF_FWD` 17.3+0.1590, `LB_FF_REV` 14.7+0.1585, `RB_FF_FWD` 14.1+0.1647, `RB_FF_REV` 14.9+0.1565; floors 70 |
| 7-8 Speed PI | Two sweeps (18 runs). Chosen for accuracy in BOTH directions. Confirm run: steady error within +-1.3 %, ripple <= 2.7 %, all steps fwd and rev | LB kp 0.045 ki 0.30; RB kp 0.0375 ki 0.40 |
| IMU under load | 322 lines with both rears at 150: no zero or stale IMU readings | - |

Also: 75 mm spiked wheels: URDF `wheel_radius` 0.0375 (was 0.0239), base_footprint height 0.1082 (was 0.0946), `my_controllers*.yaml` `wheel_radius` 0.0375 (confirm in step 12). Tools: `~/auracle_tools/ff_sweep.py`, `pid_tune.py --sweep --kp-mult ... --ki-mult ...` (smoothed, per-side ranking), `front_offset_test.py [LF RF LB RB]`.

## 4.14 Floor tests and nav tuning 2026-10-26
- Floor tests (`~/auracle_tools/calib_floor.py`, logs in `~/auracle_tools/runs/calib_floor*.log`). The LF (damaged) is bistable: it stalls below ~230 PWM on the floor and races once free. LF_MIN 60/145 -> robot curved 68 deg LEFT over 0.8 m; LF_MIN 230/255 (current) -> 33 deg RIGHT. Needs a new motor or a gyro-trimmed LF (not done).
- Effective track width: right spin with all wheels turning = 0.59 m (other spins 0.19-0.80, skewed by LF).
- EKF: yaw rate from the IMU, wheels give vx only. IMU angular-velocity covariance 0.0004.
- diff_cont: wheel_separation 0.59, linear +-0.20 m/s, accel 0.5, angular 1.0.
- Nav2: AMCL alpha1/2 0.25, goal xy 0.12, RPP 0.15 m/s, rotate 0.8 rad/s, velocity smoother 0.18 m/s, accel 0.5 / 1.5. slam_toolbox angle_variance_penalty 0.3.
- Script bug fixed: drive_test/spin_test integrated gy (pitch) instead of gz; the first straight-run heading results were invalid.

## 5. Pending / open issues

| Issue | Status / next step |
|---|---|
| **IMU not responding** | Nano A5 pin damaged (4.10). Replace the Nano, or move the IMU to software I2C on A2/A3. When fixed: re-enable `imu0` in `ekf.yaml` and set `odom0_config` vyaw back to false |
| **Pi power cuts** | Check and resolder the cut USB-C cable and the buck terminals. Raise the buck output to ~5.15 V at the Pi. Capacitors on the buck input. Or run the Pi from a USB-C power bank. `power_log.sh` records the 5 V history for the next cut. **Being undone:** `usb_max_current_enable=1` (line 2 of `/boot/firmware/config.txt`) seemed to make the shut-offs more frequent. Remove it with `sudo sed -i '/^usb_max_current_enable=1$/d' /boot/firmware/config.txt` and reboot |
| **Wi-Fi power save** | `source pc/auracle_up.sh` turns it off now and on every boot. By hand: `sudo iw dev wlan0 set power_save off`. To make it permanent: `printf '[connection]\nwifi.powersave = 2\n' \| sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf` |
| **Right rear weak forward** | Hardware: check its L298 channel and motor, or swap to MOSFET drivers. When repaired: raise `RB_FWD_MAX_TICKS_S`, restore `RB_FF_FWD_*`, retune `RF_TRIM_FWD`, then raise the forward speed limits again |
| **Right encoder dropouts** | Find the intermittent connection. When fixed: `RB_COUNT_ON_B = false` |
| **Lidar CP2102 lockups** | Replug the lidar, then kill the hung `rplidar_composition` so the launch respawns it. Try a black USB 2.0 port, or a powered hub. Replace the adapter if it continues |
| **Minimum-speed (hold) test on the floor** | Find the lowest PWM both rears hold smoothly, then lower `MIN_TARGET_TICKS_S` below 450 for gentler starts |
| **Front trims** | `RF_TRIM_FWD` 0.5 / `LF_TRIM` 1.0 are estimates. Straight-line test: drifts left → lower `RF_TRIM_FWD`, drifts right → raise it |
| **Odometry check** | Drive 1 m and spin 360° to confirm `wheel_radius` 0.033 and `wheel_separation` 0.369 |
| **A good real-arena map** | `map_1` is from the simulator, so it only localizes if the real field matches. Remap the real arena once power and Wi-Fi are fixed |
| **Nav tuning after the arc/pivot change** | Watch corner behaviour in tight corridors. If it turns too wide, lower `rotate_to_heading_min_angle` |
| **Uncommitted work** | Commit the latest firmware (arc instead of pivot) and `nav2_params.yaml` (rotate-to-heading). This file too |
