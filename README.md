# diplomski
# bachelor-project


## Getting Started

Create a workspace (only the first time):
```bash
mkdir -p eurobot_ws/src
cd eurobot_ws
git clone https://github.com/lazicdanilo/bachelor-project.git src/eurobot
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy
```

Build:
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

Run:
```bash
# eurobot simulation
ros2 launch eurobot_simulation robot_launch.py

# eurobot fsm
ros2 run eurobot_fsm eurobot_fsm
```

Play:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
