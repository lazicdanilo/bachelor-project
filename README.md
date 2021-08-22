# diplomski
# bachelor-project


## Getting Started

Create a workspace (only the first time):
```bash
mkdir -p eurobot_ws/src
cd eurobot_ws
git clone https://github.com/lazicdanilo/bachelor-project.git src/eurobot
# We use some cutting edge features from `webots_ros2` which will be available in the Debian distribution in a week or two.
# After, `sudo apt install ros-foxy-webots-ros2` will be enough.
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy
```

Build:
```bash
source /opt/ros/foxy/local_setup.bash
colcon build
source install/local_setup.sh
```

Run:
```bash
ros2 launch eurobot_simulation robot_launch.py
```

Play:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
