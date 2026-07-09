# SpotMicro ROS 2 Gazebo Simulation

This package contains a ROS 2 / Gazebo Sim setup for a SpotMicro quadruped model inspired by the original [`mike4192/spotMicro`](https://github.com/mike4192/spotMicro) project. The simulation structure and force-control pipeline were adapted using [`faoezg/QuadrupedA1Controller`](https://github.com/faoezg/QuadrupedA1Controller) as a reference.

The current project provides:

- A SpotMicro URDF/Xacro model with Gazebo-compatible mesh paths.
- A Gazebo empty world for testing the robot.
- ROS-Gazebo bridges for joint states, TF, odometry, and joint force commands.
- A pygame joint-position GUI that converts desired joint angles into bounded Gazebo joint force commands.

> Note: The direct joint GUI is intended for manual posing and simulation debugging. Full walking still needs a higher-level gait controller that produces stable joint targets.

## Workspace Setup

Create a ROS 2 workspace and clone or place this package under `src`:

```bash
mkdir -p spotmicro_ws/src
cd spotmicro_ws/src

# Place or clone your SpotMicro ROS 2 package here.
# Example if this package is available as a repository:
# git clone <your-spotmicro-description-repo-url> spotmicro_description
```

Your workspace should look similar to this:

```text
spotmicro_ws/
  src/
    spotmicro_description/
      config/
      launch/
      meshes/
      scripts/
      urdf/
      worlds/
      CMakeLists.txt
      package.xml
```

## Dependencies

Install the ROS 2 and Gazebo packages used by the simulation.

For ROS 2 Jazzy:

```bash
sudo apt-get update
sudo apt-get install \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  python3-pygame
```

If you are using another ROS 2 distribution, replace `jazzy` with your distro name.

## Build

From the workspace root:

```bash
cd ~/spotmicro_ws
colcon build --symlink-install
source install/setup.bash
```

If you delete `build/`, `install/`, or `log/`, open a new terminal or re-source after rebuilding:

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

You can check that the executable nodes were installed with:

```bash
ros2 pkg executables spotmicro_description
```

Expected executables include:

```text
spotmicro_joint_force_gui.py
```

## Launch Gazebo

Start the simulation:

```bash
cd ~/spotmicro_ws
source install/setup.bash
ros2 launch spotmicro_description sim.launch.py
```

The default world is `custom_environment.sdf`. To launch the depot world instead:

```bash
ros2 launch spotmicro_description sim.launch.py world:=depot.sdf
```

Press the play/start button in Gazebo before starting the joint GUI.

For joint testing without the body falling, launch the pinned zero-gravity test world:

```bash
ros2 launch spotmicro_description joint_test.launch.py
```

This spawns SpotMicro in the air with the base held kinematic so the GUI can move individual joints without the body dropping onto the floor.

## Move The Joints

Use separate terminals. In every new terminal, source the workspace first:

```bash
cd ~/spotmicro_ws
source install/setup.bash
ros2 run spotmicro_description spotmicro_joint_force_gui.py
```

The GUI subscribes to `/joint_states`, starts passive, and publishes zero torque until you drag a joint slider or press `Space` / `H`. The first slider movement enables a soft whole-body hold at the current measured pose, then ramps the selected joint target so commands do not step instantly. Press `M` to release all joints and match the sliders to the current measured pose.

If the robot still jitters, start softer and increase the values gradually:

```bash
ros2 run spotmicro_description spotmicro_joint_force_gui.py --ros-args \
  -p gain_scale:=0.5 \
  -p torque_scale:=0.5 \
  -p target_rate:=0.4
```

## Control Pipeline

The current manual control flow is:

```text
spotmicro_joint_force_gui.py
  -> /joint_states feedback
  -> /front_left_leg_cmd, /front_right_leg_cmd, ...
  -> ros_gz_bridge
  -> Gazebo /model/spotmicro/joint/<joint_name>/cmd_force
```

The bridge configuration is defined in:

```text
config/ros_gz_bridge.yaml
```

The Gazebo plugins for joint force commands are added from:

```text
urdf/spotmicro.ros2_control.xacro
```

## Useful Debug Commands

Check ROS topics:

```bash
ros2 topic list
```

Echo the simulated IMU mounted on `imu_link`:

```bash
ros2 topic echo /imu/data
```

Echo the simulated 2D lidar mounted on `lidar_link`:

```bash
ros2 topic echo /scan
```

Check one GUI torque command:

```bash
ros2 topic echo /front_left_leg_cmd --field data
```

Check Gazebo command topics:

```bash
gz topic -l | grep spotmicro | sort
```

Check Gazebo sensor topics:

```bash
gz topic -l | grep -E 'imu|scan'
```

Echo one Gazebo force topic directly:

```bash
gz topic -e -t /model/spotmicro/joint/front_left_leg/cmd_force
```

## Development Notes

The original SpotMicro project uses a kinematics and gait pipeline based on foot positions, body state, and servo angle conversion. This ROS 2 package currently provides direct manual joint positioning; a walking stack can build on top of the same joint limits, joint state feedback, and force command bridge.

The most important remaining tuning tasks are:

- Verify the mapping from original SpotMicro servo names (`RF_1`, `RF_2`, etc.) to this URDF's joint names.
- Verify joint signs for left and right legs.
- Tune the GUI PD gains for your Gazebo timestep and robot weight.
- Add a validated SpotMicro kinematics and gait controller on top of the manual joint interface.

## Common Issues

### `No executable found`

When using `--symlink-install`, Python scripts must have executable permissions:

```bash
chmod +x src/spotmicro_description/scripts/*.py
colcon build --symlink-install
source install/setup.bash
```

Run nodes with `ros2 run`, not `ros2 launch`:

```bash
ros2 run spotmicro_description spotmicro_joint_force_gui.py
```

### Gazebo does not move

Check that Gazebo is unpaused and that force topics exist:

```bash
gz topic -l | grep cmd_force
```

Then run the joint GUI and move a single joint slowly:

```bash
ros2 run spotmicro_description spotmicro_joint_force_gui.py
```

### Robot rocks when pygame starts

This usually means the home pose or PD gains are too aggressive for the current Gazebo timestep or robot mass. Inspect the measured position and torque values in the GUI, and echo one command topic:

```bash
ros2 topic echo /front_left_leg_cmd --field data
```

Reduce the affected joint's `kp` or `max_torque` in `scripts/spotmicro_joint_force_gui.py`, rebuild, and test again.