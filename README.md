# Building
Do this on every machine that runs the workspace (Pi, laptop, WSL). The `chmod` is needed because of `--symlink-install`.
```
chmod +x src/auracle_bringup/scripts/*.py
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```
  
Everything lives in this workspace except one Windows-only file, `joy_bridge_win.py`
(kept in `C:\my_gits\test_files`, needs `pip install pygame`).

# Virtual Teleop + SLAM
**Terminal 1 (Gazebo + robot)**
This launches the robot in Gazebo with use_sim_time:=true baked in. Build and save a map
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py use_joy_node:=false
```
`use_joy_node:=false` means no controller is read on this machine (keyboard, or the Windows bridge below). Drop it if the controller is attached to this machine.

Wait for it to settle — Gazebo window opens, spawner sequence finishes diff_cont → joint_broad → imu_broadcaster, one after another now

To confirm in a separate terminal
```
ros2 control list_controllers          # diff_cont must show "active". If any inactive, launch with ros2 run controller_manager spawner diff_cont
```
**Terminal 2 (SLAM, NAV2, Rviz)**
```
source install/setup.bash && ros2 launch auracle_bringup slam_mapping.launch.py use_sim_time:=trueuse_sim_time:=true
```
**Terminal 3 (Teleop, keyboard)**
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```
Or drive with the controller instead: see "Controller teleop" below.

**Terminal 4 (Map saving)**
This writes map_1.yaml + map_1.pgm into config/
```
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f $(pwd)/src/auracle_bringup/config/map_1
```
Add map_server block to nav2_params.yaml
```
map_server:
  ros__parameters:
    yaml_filename: ""
```

## Navigation

**Terminal 1 (simulation)**
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py use_joy_node:=false
```

**Terminal 2 (AMCL + Nav2 + RViz, against your saved map)**
```
source install/setup.bash && ros2 launch auracle_bringup localization_nav_rviz.launch.py use_sim_time:=true map:=$(pwd)/src/auracle_bringup/config/map_1.yaml
```

In RViz, check that:
the global costmap now covers the whole map;
the particle cloud is a tight cluster around the robot;
there are no TF errors in the Displays panel.
Click Nav2 Goal in the toolbar, then click-and-drag on a free (grey) spot of the map. The click sets the position and the drag sets the heading.
A green line (GlobalPlan) should appear and the robot should start driving in Gazebo. Navigation goes through cmd_vel_nav into twist_mux at priority 40, which is wired correctly.
Don’t press keys in the teleop window while a goal is running, because joystick and teleop input has priority 100 and overrides Nav2.


# Real robot (Pi 5 + Arduino Nano)

## Flashing the Nano from WSL and platformIO. (See platform.ini for ArduinoIDE)
PowerShell as admin, once per device:
```
usbipd list                      # Nano = FTDI 0403:6001, e.g. BUSID 1-2
usbipd bind --busid=1-2
```
Every time you replug:
```
usbipd attach --wsl --busid=1-2
```
In WSL:
```
ls /dev/ttyUSB*                  # FTDI shows up as /dev/ttyUSB0
cd firmware/auracle_firmware
pio run -t upload --upload-port /dev/ttyUSB0
```
To flash from the Pi instead, use the same command with `--upload-port /dev/arduino`. Close ROS first, because only one program can hold the port.

## Installations
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-localization ros-jazzy-twist-mux ros-jazzy-twist-stamper ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher ros-jazzy-joy ros-jazzy-rmw-zenoh-cpp libserial-dev
```


## Adding udev rules
```
#check if rules file exists
ls /etc/udev/rules.d/99-auracle.rules
#if missing, copy from workspace
sudo cp src/auracle_bringup/config/99-auracle.rules /etc/udev/rules.d/
```
Reload and trigger the udev rules to generate the symlinks. Physical connection: Both on blue ports, lidar on top of arduino
```
sudo udevadm control --reload-rules
sudo udevadm trigger
#Verify the symlinks now exist and point to the correct ttyUSB ports
ls -l /dev/arduino /dev/rplidar
#ensure your user account is in the dialout group to allow serial access:
sudo usermod -aG dialout $USER
#log out and log back into your Ubuntu session for the group changes to take effect.
sudo reboot
```

## Pi <-> laptop networking (Zenoh)
Every machine needs the same RMW and domain, e.g. in `~/.bashrc` (fill in your own values):
```
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=<same number on every machine>
export ZENOH_SESSION_CONFIG_URI=$HOME/zenoh_config.json5   # mode "client", endpoints = the routers
```
A router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) must be running on the machines listed as endpoints.
For a single-machine sim with no reachable routers, start a local router and point
`ZENOH_SESSION_CONFIG_URI` at a config whose only endpoint is `tcp/127.0.0.1:7447`.
Check the link: `ros2 topic list | grep auracle` on the laptop while the Pi launch is running.

## Bring-up

**On Pi Terminal: robot (hardware interface, controllers, lidar, EKF)**
```
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py namespace:=auracle use_joy_node:=false
```
If lidar not connected, skip it:
```
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py use_lidar:=false namespace:=auracle use_joy_node:=false
```
`use_joy_node:=false` is right whenever `/joy` comes from another machine (Windows bridge or laptop). Drop it only if the controller is paired to the Pi itself.

To confirm in a separate terminal
```
ros2 control list_controllers -c /auracle/controller_manager         # diff_cont must show "active". If any inactive, launch with ros2 run controller_manager spawner diff_cont
```

Confirm topics and services are live with `ros2 topic list` and `ros2 service list`

**Terminal 2: keyboard teleop**
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/auracle
```

**SLAM + RViz** (run on the laptop, same `ROS_DOMAIN_ID`)
```
source install/setup.bash && ros2 launch auracle_bringup slam_mapping.launch.py namespace:=auracle
```

# Controller teleop

The drive node (`joy_drive.py`) runs where `launch_robot` / `launch_sim` runs, so on the Pi for the real robot.
The other machine only has to supply `/joy`. The namespace must match the robot: `/auracle` on the robot, none in the sim.
Smoothing, speed and the button mapping are all in `config/joystick.yaml`.

## Controls
| Input | Does |
|---|---|
| LT + RT | Both pressed drives forward. Release LT and it banks left, release RT and it banks right. |
| D-pad up / down | Speed level up / down in 10% steps (hold to repeat). Scales everything, starts at 50%. |
| Y, A | Forward, backward |
| X, B | Turn left, turn right. Combine with Y or A for arcs. |
| LB + left stick | Analog driving. LB is the dead-man for the stick. |

All schemes work at the same time. For forward and for turning, whichever input is pushed hardest wins.
On the 8BitDo Ultimate 2C over Bluetooth, LT/RT are on/off, not analog: they ease in smoothly, and speed comes from the d-pad level.

## Option A: controller paired to Windows, bridged into WSL
1. Pair the controller in Windows Bluetooth settings.
2. Pi: run the Bring-up command above (`namespace:=auracle use_joy_node:=false`).
3. WSL (with your normal Zenoh config, `ros2 topic list | grep auracle` must show topics):
```
source install/setup.bash && ros2 run auracle_bringup joy_udp_node.py --ros-args -r __ns:=/auracle
```
4. Windows PowerShell:
```
cd C:\my_gits\test_files
python joy_bridge_win.py --host 100.81.100.9           # python joy_bridge_win.py --probe  shows what each button maps to
```
In the sim, the same but with no namespace: `ros2 run auracle_bringup joy_udp_node.py`.

## Option B: controller paired to a native Linux laptop
Pair once:
```
bluetoothctl
> power on
> agent on
> default-agent
> scan on            # controller in pairing mode
> pair XX:XX:XX:XX:XX:XX
> trust XX:XX:XX:XX:XX:XX
> connect XX:XX:XX:XX:XX:XX
```
Check the numbers with `ros2 run joy joy_node` and `ros2 topic echo /joy`: LB must be `buttons[4]`, left-stick-up must make `axes[1]` positive. If not, edit the mapping in `config/joystick.yaml` **on the Pi** (the drive node reads it there).
Then on the laptop, publish only `/joy` and leave the driving to the Pi:
```
source install/setup.bash && ros2 launch auracle_bringup joystick.launch.py namespace:=auracle use_drive_node:=false
```

## Option C: controller paired to the Pi
Launch the robot without `use_joy_node:=false`; it reads the controller itself.

## Before driving
Put the robot on blocks, wheels off the floor, and check:
- Hold LT + RT, then Ctrl-C the bridge: the wheels stop within about a second.
- Hold LT + RT, then switch the controller off: note how long the wheels keep going.
- Hold LT + RT, then cut the network: the wheels stop.

** Save the map **
```
ros2 run nav2_map_server map_saver_cli -f ~/map_2 --ros-args -r map:=/auracle/map
```

# navigation againts saved 
ros2 launch auracle_bringup localization_nav_rviz.launch.py namespace:=auracle \
  map:=~/map_2.yaml