# bachelor-project

## Getting Started

### Install ROS2 Foxy
Use the tutorial from [this](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) link

### Install Webots
```bash
sudo snap install webots
```

### Create a workspace (only the first time):
```bash
mkdir -p eurobot_ws/src
cd eurobot_ws
git clone https://github.com/lazicdanilo/bachelor-project.git src/eurobot
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy
```
### Install dependencies (For Ubuntu)
```bash
sudo apt install python3-rosdep2 install python3-colcon-ros ros-foxy-webots-ros2-driver ros-foxy-webots-ros2-control python3-opencv
```

### Build
```bash
# only eurobot simulation
source /opt/ros/foxy/local_setup.bash
colcon build --packages-select eurobot_fsm
source install/local_setup.sh

# only eurobot fsm
source /opt/ros/foxy/local_setup.bash
colcon build --packages-select eurobot_fsm
source install/local_setup.sh

# both
source /opt/ros/foxy/local_setup.bash
colcon build
source install/local_setup.sh
```

### Run
```bash
# eurobot simulation
source /opt/ros/foxy/local_setup.bash
source install/local_setup.sh
ros2 launch eurobot_simulation robot_launch.py

# eurobot fsm
source /opt/ros/foxy/local_setup.bash
source install/local_setup.sh
ros2 run eurobot_fsm eurobot_fsm
```

### Play
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
