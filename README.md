# TurtleBot3 Trajectory Controller

A ROS2 package for autonomous trajectory following using cubic spline path smoothing and pure pursuit control.

## Features
- Multiple path scenarios (straight line, zigzag, S-curve)
- Cubic spline path smoothing
- Pure pursuit control algorithm
- Real-time RViz visualization

## Quick Setup

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-turtlebot3* ros-humble-gazebo-* python3-pip
pip3 install scipy numpy
```

### 2. Environment Setup
```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 3. Build Package
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <your-repository-url> trajectory_controller
cd ~/ros2_ws
colcon build --packages-select trajectory_controller
source install/setup.bash
```

## Usage

### Terminal 1: Launch Simulation
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Terminal 2: Run Trajectory Controller
```bash
cd ~/ros2_ws && source install/setup.bash
# Choose one scenario:
ros2 launch trajectory_controller trajectory_nodes.launch.py scenario:=straight_line
ros2 launch trajectory_controller trajectory_nodes.launch.py scenario:=zigzag
ros2 launch trajectory_controller trajectory_nodes.launch.py scenario:=s_curve
```

### Terminal 3: Visualization
```bash
rviz2
```

## RViz Setup
1. Set **Fixed Frame** to `odom`
2. Add displays for:
   - **RobotModel**
   - **Path** for `/waypoints`, `/smooth_path`, `/trajectory`
   - **Odometry** for `/odom`
   - **TF**

## Troubleshooting

### Robot Not Moving
```bash
# Check odometry
ros2 topic echo /odom

# Verify TF frames
ros2 run tf2_tools view_frames.py

# Check topics
ros2 topic list
ros2 topic echo /cmd_vel
```

### Build Issues
```bash
# Clean build
rm -rf build install log && colcon build
source install/setup.bash
```

### Path Not Visible
- Verify topics: `ros2 topic echo /waypoints`
- Check RViz Fixed Frame is set to `odom`
- Ensure path displays are properly configured

## Manual Node Execution (Debug)
```bash
# Terminal 1
ros2 run trajectory_controller test_scenarios_node --ros-args -p scenario:=zigzag

# Terminal 2
ros2 run trajectory_controller path_smoother_node

# Terminal 3
ros2 run trajectory_controller trajectory_generator_node

# Terminal 4
ros2 run trajectory_controller pure_pursuit_node
```


## Project Structure
```
trajectory_controller/
├── launch/trajectory_nodes.launch.py
├── trajectory_controller/
│   ├── test_scenarios.py
│   ├── path_smoother.py
│   ├── trajectory_generator.py
│   └── pure_pursuit_controller.py
├── package.xml
└── setup.py
```
