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

```
ros2 launch fred2_controllers position_controller.launch.py
```

--- 

## Position Control Node
The positionController node implements a position control algorithm using a PID controller. It subscribes to odometry and goal information, calculates the error between the current position and the goal, and outputs velocity commands to achieve the desired position. The node is capable of handling both MOVE BASE and ROBOT LOCALIZATION odometry sources.

One notable feature of the `positionController` node is its capability to dynamically switch the robot's orientation for improved trajectory performance. The node intelligently determines whether to move forward or backward based on the orientation error, optimizing the robot's movement towards the goal.


---

### Parameters: 

- `kp_angular`: Proportional gain for angular control in the PID algorithm.

- `ki_angular`: Integral gain for angular control in the PID algorithm.

- `kd_angular`: Derivative gain for angular control in the PID algorithm.

- `kp_linear`: Proportional gain for linear control in the PID algorithm.

- `ki_linear`: Integral gain for linear control in the PID algorithm.

- `kd_linear`: Derivative gain for linear control in the PID algorithm.

- `max_linear_vel`: Maximum linear velocity allowed for the robot.

- `min_linear_vel`: Minimum linear velocity allowed for the robot.

- `debug`: Enables the debug prints.

- `unit_test`: Allows the node to run isolated .


#### To check available parameters 
```
ros2 param list 
```

#### To check the parameter description
```
ros2 param describe /controllers/positionController <parameter name>

```

#### To check the parameter value
```
ros2 param get /controllers/positionController <parameter name>
```

#### To reload the parameters file
```
ros2 param load /controllers/positionController <path-to-parameter-config-file>
```

---

### Topics

- **Subscribers:**

  - `/odom` (*nav_msgs/Odometry*): Odometry information from the robot

  - `/goal_manager/goal/current` (*geometry_msgs/PoseStamped*): Current goal information.

  - `/machine_states/robot_state` (*std_msgs/Int16*): Robot's state information.

- **Publisher:**

  - `/cmd_vel`: Velocity commands for robot movement.

--- 

### Run 

**Default:**

```
ros2 run fred2_controllers positionController --ros-args --params-file /home/ubuntu/ros2_ws/src/fred2_controllers/config/controllers_params.yaml
 
```

**Enable debug:**
```
ros2 run fred2_controllers positionController --debug
```
