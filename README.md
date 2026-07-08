# SpotMicro ROS 2 Gazebo Simulation

This package contains a ROS 2 / Gazebo Sim setup for a SpotMicro quadruped model inspired by the original [`mike4192/spotMicro`](https://github.com/mike4192/spotMicro) project. The simulation structure and force-control pipeline were adapted using [`faoezg/QuadrupedA1Controller`](https://github.com/faoezg/QuadrupedA1Controller) as a reference.

The current project provides:

- A SpotMicro URDF/Xacro model with Gazebo-compatible mesh paths.
- A Gazebo empty world for testing the robot.
- ROS-Gazebo bridges for joint states, TF, odometry, and joint force commands.
- A low-level PD controller that converts desired joint positions into Gazebo joint force commands.
- Experimental high-level, crawl, pose, joystick, and diagnostic controllers.

> Note: The Gazebo topic bridge is functional, but the locomotion controller is still experimental. The main remaining work is tuning the SpotMicro-specific kinematics, joint sign mapping, and gait conversion so `/cmd_vel` commands produce stable walking.

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
spotmicro_cmdvel_pub_gui.py
spotmicro_crawl_controller.py
spotmicro_force_test.py
spotmicro_high_level_controller.py
spotmicro_joint_force_gui.py
spotmicro_low_level_controller.py
spotmicro_odometry_publisher.py
spotmicro_pose_controller.py
spotmicro_pose_pub_gui.py
```

## Launch Gazebo

Start the simulation:

```bash
cd ~/spotmicro_ws
source install/setup.bash
ros2 launch spotmicro_description sim.launch.py
```

Press the play/start button in Gazebo before starting the controllers.

## Run The Controller Stack

Use separate terminals. In every new terminal, source the workspace first:

```bash
cd ~/spotmicro_ws
source install/setup.bash
```

Start the low-level controller:

```bash
ros2 run spotmicro_description spotmicro_low_level_controller.py
```

Start the high-level controller:

```bash
ros2 run spotmicro_description spotmicro_high_level_controller.py
```

Start the pygame `/cmd_vel` GUI:

```bash
ros2 run spotmicro_description spotmicro_cmdvel_pub_gui.py
```

The joystick publishes `geometry_msgs/msg/Twist` on `/cmd_vel`. The high-level controller converts `/cmd_vel` into desired joint positions on `/high_level/joint_cmd`. The low-level controller converts those desired positions into Gazebo force commands for each joint.

## Control Pipeline

The current control flow is:

```text
pygame GUI
  -> /cmd_vel
  -> spotmicro_high_level_controller.py
  -> /high_level/joint_cmd
  -> spotmicro_low_level_controller.py
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

Check the joystick command:

```bash
ros2 topic echo /cmd_vel
```

Check desired joint positions from the high-level controller:

```bash
ros2 topic echo /high_level/joint_cmd
```

Check one low-level torque command:

```bash
ros2 topic echo /front_left_leg_cmd --field data
```

Check Gazebo command topics:

```bash
gz topic -l | grep spotmicro | sort
```

Echo one Gazebo force topic directly:

```bash
gz topic -e -t /model/spotmicro/joint/front_left_leg/cmd_force
```

## Direct Force Test

To isolate Gazebo and the ROS-Gazebo bridge from the high-level controller, stop the low-level controller and run:

```bash
ros2 run spotmicro_description spotmicro_force_test.py
```

You can select a joint and amplitude:

```bash
ros2 run spotmicro_description spotmicro_force_test.py \
  --ros-args \
  -p topic:=/front_left_leg_cmd \
  -p amplitude:=120.0 \
  -p frequency:=0.5
```

If the selected joint moves, Gazebo and the bridge are receiving force commands correctly. If it does not move, check that Gazebo is unpaused and that the `cmd_force` topics exist.

## Development Notes

The original SpotMicro project uses a kinematics and gait pipeline based on foot positions, body state, and servo angle conversion. This ROS 2 simulation currently contains a partial Python adaptation in:

```text
scripts/spotmicro_reference.py
```

The most important remaining tuning tasks are:

- Verify the mapping from original SpotMicro servo names (`RF_1`, `RF_2`, etc.) to this URDF's joint names.
- Verify joint signs for left and right legs.
- Tune the low-level PD gains for Gazebo dynamics.
- Replace experimental gait approximations with a fully validated SpotMicro kinematics port.

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
ros2 run spotmicro_description spotmicro_low_level_controller.py
```

### Gazebo does not move

Check that Gazebo is unpaused and that force topics exist:

```bash
gz topic -l | grep cmd_force
```

Then run the direct force test:

```bash
ros2 run spotmicro_description spotmicro_force_test.py
```

### Robot rocks when pygame starts

This means the high-level controller is producing joint targets that do not match the stable Gazebo posture. Inspect:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /high_level/joint_cmd
ros2 topic echo /front_left_leg_cmd --field data
```

The likely fix is to adjust joint sign mapping and the SpotMicro kinematics conversion in `scripts/spotmicro_reference.py`.