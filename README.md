# ros2-gazebo-ball-beam-lqr
Ball-beam control simulation in ROS 2 Jazzy and Gazebo Harmonic using an LQR controller for real-time stabilization.

# Ball-Beam LQR Controller — ROS 2 Jazzy + Gazebo Harmonic

This project implements a simulated **ball-beam balancing system** in **ROS 2 Jazzy** and **Gazebo Harmonic**. The beam is position-controlled, while a Python ROS 2 node computes an **LQR control command** to keep the ball near the center of the beam.

## Project Overview

The controller uses the state vector:

```text
x = [ball_position, ball_velocity, beam_angle, beam_angular_velocity]
```

The control input is:

```text
u = desired beam angle [rad]
```

The linearized ball-beam model is:

```text
x_ball_ddot = -(5/7) g theta_beam
```

A continuous-time LQR gain is computed using the Continuous Algebraic Riccati Equation (CARE). The resulting control law drives the beam command based on the measured ball and beam states.

## Features

- ROS 2 Python LQR controller node
- Gazebo Harmonic simulation support
- URDF/Xacro model of the ball-beam system
- `ros2_control` integration through `gz_ros2_control`
- Joint state feedback from `/joint_states`
- Beam position command publishing to `/beam_position_controller/commands`
- Velocity smoothing to reduce jitter
- Dead-band near equilibrium to avoid unnecessary actuation
- Command clipping to respect beam joint limits

## Recommended Repository Structure

Use this structure before uploading to GitHub:

```text
ball_balancer/
├── README.md
├── .gitignore
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── ball_balancer
├── ball_balancer/
│   ├── __init__.py
│   └── lqr_controller.py
├── config/
│   └── controllers.yaml
├── urdf/
│   └── ball_beam.urdf.xacro
├── launch/
│   └── ball_beam_sim.launch.py
└── media/
    └── demo.gif
```

Only keep one main controller file in the package. The recommended final file is:

```text
ball_balancer/lqr_controller.py
```

Older experimental controller versions can be kept in an `archive/` folder or left out of the public repository.

## What to Upload

Upload these files/folders:

```text
README.md
package.xml
setup.py
setup.cfg
resource/ball_balancer
ball_balancer/__init__.py
ball_balancer/lqr_controller.py
config/controllers.yaml
urdf/ball_beam.urdf.xacro
launch/*.py
media/demo.gif or screenshots, if available
.gitignore
```

Do **not** upload generated ROS 2 build folders:

```text
build/
install/
log/
__pycache__/
*.pyc
```

## Dependencies

This project expects a ROS 2 Jazzy workspace with Gazebo Harmonic support.

Typical dependencies include:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-position-controllers \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  python3-numpy \
  python3-scipy
```

## Installation

Create or use an existing ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/ball_balancer.git
```

Build the workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

If a launch file is included:

```bash
ros2 launch ball_balancer ball_beam_sim.launch.py
```

Spawn the controllers if they are not already started by the launch file:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner beam_position_controller
```

Run the LQR controller:

```bash
ros2 run ball_balancer lqr_controller
```

## Controller Design

The controller computes the LQR gain from the linearized model:

```python
A = [[0, 1,      0, 0],
     [0, 0, -5g/7, 0],
     [0, 0,      0, 1],
     [0, 0,      0, 0]]

B = [[0],
     [0],
     [0],
     [1]]
```

The cost function is:

```text
J = integral( xᵀQx + uᵀRu ) dt
```

where:

- `Q` penalizes ball position error, velocity, beam angle, and beam angular velocity
- `R` penalizes aggressive beam commands
- `K` is computed from the Riccati equation
- the beam command is generated from the state error

## Topics

`/joint_states`

Used by the controller to read:

- `ball_joint` position and velocity
- `beam_joint` position and velocity

`/beam_position_controller/commands`

Used by the controller to publish:

- desired beam angle command as `Float64MultiArray`

## Notes

- The URDF/Xacro model uses `ros2_control` and the `gz_ros2_control` plugin.
- The beam joint is position-controlled.
- The ball joint is passive and provides state feedback.
- The controller includes a Gazebo sign correction because the simulation sign convention can differ from the textbook model.
- The beam command is clipped near the URDF joint limit to avoid unrealistic actuation.

## Future Improvements

- Add a launch file that starts Gazebo, robot description, controller manager, and the LQR node together.
- Add a setpoint topic or parameter for changing desired ball position.
- Add plots for ball position, beam angle, and control effort.
- Add a demo GIF or video in the README.
- Compare LQR performance with PID control.
- Add automated tests for the LQR gain computation.

## License

Add a license before publishing the repository. For academic/demo projects, MIT License is a common choice.
