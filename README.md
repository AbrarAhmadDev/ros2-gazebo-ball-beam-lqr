# ros2-gazebo-ball-beam-lqr
Ball-beam control simulation in ROS 2 Jazzy and Gazebo Harmonic using an LQR controller for real-time stabilization.

# Ball-Beam LQR Controller — ROS 2 Jazzy + Gazebo Harmonic

## 1. Requirements

Tested for:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12

Install required packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro \
  ros-jazzy-rqt \
  ros-jazzy-rqt-graph \
  python3-colcon-common-extensions \
  python3-pip
```

Install Python libraries:

```bash
pip3 install numpy scipy pandas matplotlib
```

## 2. Required Project Files/Folders

Project should include:

```text
ros2_ws/
└── src/
    └── ball_balancer/
        ├── ball_balancer/
        │   ├── __init__.py
        │   └── lqr_controller.py
        ├── config/
        │   └── controllers.yaml
        ├── launch/
        │   └── simulation.launch.py
        ├── urdf/
        │   └── ball_beam.urdf.xacro
        ├── resource/
        │   └── ball_balancer
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

## 3. Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ball_balancer
source install/setup.bash
```

## 4. Run the Project

### Terminal 1 — Start Gazebo Simulation

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
ros2 launch ball_balancer simulation.launch.py
```

Wait until Gazebo opens and the controllers load. (Ideally wait 15 to 20 seconds.)

### Terminal 2 — Run LQR Controller

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ball_balancer lqr_controller
```

The controller subscribes to:

```text
/joint_states
```

and publishes desired beam angle commands to:

```text
/beam_position_controller/commands
```

## 5. Optional: Manual Beam Command Test

Stop the LQR controller first, then run:

```bash
ros2 topic pub --once /beam_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.20]}"
```

## 6. Check Controllers

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster active
beam_position_controller active
```

## 7. Open RQT Graph

### Terminal 3 — RQT Graph

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rqt_graph rqt_graph
```

## 8. Generate Plots and Metrics

After running the simulation, the controller should generate:

```text
~/ros2_ws/ball_beam_log.csv
```

Run the plotting script if included:

```bash
cd ~/ros2_ws
python3 plot_ball_beam_results.py
```

Expected output folder:

```text
~/ros2_ws/ball_beam_plots/
```

Expected generated files:

```text
ball_position.png
ball_velocity.png
tracking_error.png
beam_command.png
phase_portrait.png
performance_metrics.txt
```

## 9. Troubleshooting

If the command topic has no subscriber:

```bash
ros2 topic list | grep beam
ros2 control list_controllers
```

If the controller is inactive, activate it:

```bash
ros2 control switch_controllers --activate beam_position_controller
```

If Python cannot find `scipy` or `numpy`:

```bash
pip3 install numpy scipy
```

If changes do not appear after editing code:

```bash
cd ~/ros2_ws
colcon build --packages-select ball_balancer
source install/setup.bash
```
