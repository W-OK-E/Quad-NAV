# Instructions for Quad-NAV
### Run the following Commands

```bash

sudo apt install -y python3-rosdep
rosdep update

mkdir -p ~/Quad-NAV/src
cd ~/Quad-NAV/src
git clone https://github.com/anujjain-dev/unitree-go2-ros2.git
rosdep install --from-paths src --ignore-src -r -y

sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control

cd ~/Quad-NAV/
colcon build

cd src/
rm -rf unitree-go2-ros2.git
