# Fred - Controllers

**note: Package for ROS 2 - Python based package**

The Controllers package provides various controllers for the robot, including a position controller that utilizes a PID algorithm for controlling the robot's position. The primary node in this package is `positionController`, which is responsible for managing the robot's movement towards a goal position.

--- 

## Installation 

**1. Clone the repository into your ROS2 workspace:**

```bash
cd ros2_ws/src
git clone https://github.com/AMR-Frederico/fred2_controllers.git
```

**2. Build the package:**

```bash
cd ros2_ws
colcon build
```

**3. Install the `transforms3d` library:**

```bash
pip install transforms3d
```

----

## Usage

### Launch

**Considering `Robot Localization` for odometry and `Robot Descriptor` for publish the TFs:**
```
ros2 launch fred2_controllers position_for_robot_localization.launch.py
```

**Considering `Move Base Odometry` to publish odom and TF:**
```
ros2 launch fred2_controllers position_for_move_base_odom.launch.py
```

--- 

## Position Control Node
The positionController node implements a position control algorithm using a PID controller. It subscribes to odometry and goal information, calculates the error between the current position and the goal, and outputs velocity commands to achieve the desired position. The node is capable of handling both MOVE BASE and ROBOT LOCALIZATION odometry sources.

One notable feature of the `positionController` node is its capability to dynamically switch the robot's orientation for improved trajectory performance. The node intelligently determines whether to move forward or backward based on the orientation error, optimizing the robot's movement towards the goal.


---

### Parameters: 

- `KP_angular`: Proportional gain for angular control in the PID algorithm.

- `KI_angular`: Integral gain for angular control in the PID algorithm.

- `KD_angular`: Derivative gain for angular control in the PID algorithm.

- `max_linear_vel`: Maximum linear velocity allowed for the robot.

- `min_linear_vel`: Minimum linear velocity allowed for the robot.


---

### Topics

- **Subscribers:**

  - `/odometry/filtered` (or `/odom`) (*nav_msgs/Odometry*): Odometry information from the robot (MOVE BASE or ROBOT LOCALIZATION).

  - `/goal_manager/goal/current` (*geometry_msgs/PoseStamped*): Current goal information.

  - `/machine_states/robot_state` (*std_msgs/Int16*): Robot's state information.

- **Publisher:**

  - `/cmd_vel`: Velocity commands for robot movement.

--- 

### Run 

**Default:**

```
ros2 run fred2_controllers positionController
```

**Enable debug:**
```
ros2 run fred2_controllers positionController --debug
```
