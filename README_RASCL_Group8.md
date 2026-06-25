# RASCL-Bot ROS 2 Control Workspace

This repository contains the ROS 2 workspace for controlling the RASCL-Bot with `ros2_control`. It includes the robot description package, the hardware interface package, and the EtherCAT/Faulhaber bridge used to communicate with the real robot.

The workspace is intended to be used inside the provided Docker environment.

---

## 1. Package Overview

```text
.
├── Dockerfile
├── rosws.sh
├── README.md
└── src
    ├── rascl_description
    │   ├── config/controllers.yaml
    │   ├── launch/ros2_control.launch.py
    │   ├── rviz/urdf.rviz
    │   └── urdf/rascl.urdf
    └── rascl_hardware_interface
        ├── include/rascl_hardware_interface/
        ├── scripts/rascl_faulhaber_bridge.py
        └── src/rascl_hardware_interface.cpp
```

Main components:

- `rascl_description`: URDF, meshes, controller configuration, launch files, and RViz configuration.
- `rascl_hardware_interface`: ROS 2 hardware interface plugin and Python Faulhaber/EtherCAT bridge.
- `rosws.sh`: helper script for starting the Docker container.
- `Dockerfile`: Docker environment for ROS 2 Jazzy development and execution.

---

## 2. Starting the Docker Environment

Open a terminal in the root directory of this repository.

First make the workspace script executable:

```bash
chmod +x rosws.sh
```

Then start the container:

```bash
./rosws.sh
```

If the Docker image does not exist yet, `rosws.sh` will build it automatically. If the Dockerfile or the container dependencies were changed, the image can be rebuilt manually with:

```bash
REBUILD=true ./rosws.sh
```

A rebuild can take several minutes. If the image already exists and no Dockerfile changes were made, it is usually not necessary to rebuild.

The default Docker image/container name is:

```text
ros2-irs-rascl-wp22
```

---

## 3. Building the ROS 2 Workspace

Inside the container, build the workspace with:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

The `ROS_DOMAIN_ID=88` setting is used to isolate this ROS 2 session from old or unrelated ROS 2 processes that may still be running on the lab computers. All terminals used for this robot session must use the same `ROS_DOMAIN_ID`.

A successful build should show the RASCL packages when running:

```bash
ros2 pkg list | grep rascl
```

Expected packages:

```text
rascl_description
rascl_hardware_interface
```

---

## 4. Launching the Real Robot

After building and sourcing the workspace, launch the real robot with:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false
```

### Network Interface

If the EtherCAT network interface is not called `robot_interface`, replace it with the actual interface name.

Check available interfaces with:

```bash
ip link
```

Example:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0256dec \
  use_fake_hardware:=false
```

The encoder conversion parameters are already set as defaults in the launch/URDF files:

```text
axis_counts_per_revolution    = 3211264
gripper_counts_per_revolution = 1323008
```

Therefore they do not have to be passed manually during launch.

---

## 5. Bridge Script Permission Troubleshooting

If the launch fails with an error similar to:

```text
executable 'rascl_faulhaber_bridge.py' not found
```

or if the bridge script cannot be executed, ensure that the Python bridge has executable permission:

```bash
chmod +x src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

Then rebuild and source again:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

This is important because the launch file starts `rascl_faulhaber_bridge.py` as a ROS 2 executable.

---

## 6. Opening a Second Terminal in the Same Container

After the main launch is running, open a second terminal on the host machine and enter the same container:

```bash
docker exec -it ros2-irs-rascl-wp22 bash
```

If another container name is used, check it with:

```bash
docker ps
```

Inside the second container terminal, source the workspace and set the same domain ID:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 daemon stop
ros2 daemon start
```

Make sure that `ROS_DOMAIN_ID=88` is set in every terminal that communicates with the running ROS 2 system.

---

## 7. Reading the Current Joint Positions

The command topic uses the following joint order:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

The `upperarm_joint` is the joint close to the shoulder.

All joint positions are expressed in radians.

To print the current joint positions in the correct command order, run:

```bash
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import JointState

order = ["shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint"]

rclpy.init()
node = rclpy.create_node("print_current_order")

def cb(msg):
    d = dict(zip(msg.name, msg.position))
    print([d[j] for j in order])
    rclpy.shutdown()

node.create_subscription(JointState, "/joint_states", cb, 10)
rclpy.spin(node)
PY
```

The order printed by `/joint_states` itself may differ. Always use the joint names, not the line order, when interpreting `/joint_states`.

---

## 8. Moving the Robot

The position controller listens on:

```text
/rascl_position_controller/commands
```

The command type is:

```text
std_msgs/msg/Float64MultiArray
```

Command format:

```bash
ros2 topic pub --once /rascl_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [shoulder, upperarm, lowerarm, spur_gear]}"
```

Example:

```bash
ros2 topic pub --once /rascl_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0]}"
```

The values are absolute target positions in radians, not relative increments. Use the current joint positions as reference and command only reasonable, safe values. Avoid large jumps, especially when the robot is close to the table or other obstacles.

Recommended workflow:

1. Read the current joint positions.
2. Change only one joint by a small amount.
3. Observe the real robot and RViz.
4. Continue with small, safe changes.

---

## 9. Setting and Returning to Home

Move the robot carefully to the desired home pose using the position command topic.

Then set the current pose as the home position:

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

This uses the Faulhaber homing function to define the current motor positions as zero. After this service call, the current joint position should correspond to:

```text
[0, 0, 0, 0]
```

You can verify this with the joint-state printing command from Section 7.

To return to the home position later, publish:

```bash
ros2 topic pub --once /rascl_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 0]}"
```

Important: this home is a software/controller home defined during the current setup. It is not an absolute mechanical home unless the robot has been referenced with suitable hardware reference sensors.

---

## 10. Fault Recovery and Emergency Cleanup

If the robot collides with an obstacle or the drive enters a fault state, ROS 2 processes may stop or the launch may exit.

In this case:

1. Stop the running launch if it is still active.
2. Turn off the robot power.
3. Wait at least 10 seconds.
4. Clean up old ROS 2 processes.
5. Start again from the normal launch procedure.

Emergency cleanup commands:

```bash
pkill -9 -f rascl_faulhaber_bridge.py
pkill -9 -f ros2_control_node
pkill -9 -f controller_manager
pkill -9 -f spawner
pkill -9 -f robot_state_publisher

ros2 daemon stop
ros2 daemon start
```

Then power the robot on again and repeat the build/source/launch steps if necessary.

Do not manually backdrive the motors unless instructed by the lab supervisors.

---

## 11. RViz Visualization

The RViz model can be used to visualize the robot state in real time. The model follows the ROS 2 `/joint_states` topic and the URDF published on `/robot_description`.

### Host-Side X11 Permission

Before starting RViz from inside Docker, run the following commands on the host machine:

```bash
xhost +local:root
xhost +local:docker
```

The Docker script already forwards the display and sets `QT_X11_NO_MITSHM=1`. If RViz still cannot open, also run inside the container:

```bash
export QT_X11_NO_MITSHM=1
```

### Start RViz

In a container terminal with the same `ROS_DOMAIN_ID`, run:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

rviz2 -d src/rascl_description/rviz/urdf.rviz
```

If the model does not appear, check the `RobotModel` display settings in RViz:

```text
Description Source: Topic
Description Topic: /robot_description
```

The fixed frame should normally be:

```text
world
```

or, if needed:

```text
base_link
```

The initial RViz pose is defined by the URDF zero pose and the current `/joint_states`. Use the homing procedure to align the real robot and the RViz model as closely as possible.

---

## 12. Fake Hardware Mode

For testing the URDF, controllers, and RViz without the real robot, launch fake hardware:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  use_fake_hardware:=true
```

In fake hardware mode, the EtherCAT bridge is not started.

---

## 13. Useful Debug Commands

List controllers:

```bash
ros2 control list_controllers
```

List hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

Echo joint states:

```bash
ros2 topic echo /joint_states
```

Check running ROS nodes:

```bash
ros2 node list
```

Check available topics:

```bash
ros2 topic list
```

Check whether the bridge service exists:

```bash
ros2 service list | grep rascl_faulhaber_bridge
```

---

## 14. Optional Automated Hardware Interface Test

This repository contains an optional automated test for the `rascl_hardware_interface` package:

```text
test/test_generic_system.cpp
```

The test is intended for software-side validation only. It does not connect to EtherCAT, does not start the Faulhaber bridge, does not enable the motors, and does not move the real robot. It checks the basic `ros2_control` hardware interface behavior in fake-hardware mode, including initialization, lifecycle transitions, exported state/command interfaces, and selected invalid-configuration cases.

Normal operation and submission builds can still be performed with testing disabled:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

To build and run the optional automated test, enable `BUILD_TESTING` for the hardware interface package:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install \
  --packages-select rascl_hardware_interface \
  --cmake-args -DBUILD_TESTING=ON

source install/local_setup.bash

colcon test --packages-select rascl_hardware_interface \
  --ctest-args -R test_generic_system --output-on-failure

colcon test-result --verbose
```

A successful run should report no errors or failures, for example:

```text
Summary: x tests, 0 errors, 0 failures, 0 skipped
```

If the test dependency is missing in a clean container, check whether `ament_cmake_gtest` is installed:

```bash
dpkg -l | grep ros-jazzy-ament-cmake-gtest
```

If there is no output, install the package inside the Docker image or add it to the Dockerfile dependency list:

```text
ros-jazzy-ament-cmake-gtest
```

The automated test is not a replacement for real-hardware validation. It only verifies that the hardware interface can be built and exercised in a controlled fake-hardware setup. Real EtherCAT communication, motor direction, joint calibration, homing behavior, and mechanical safety still have to be validated on the physical RASCL robot.
