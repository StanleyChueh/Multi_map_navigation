# Multi_map_navigation
In this project, we aim to navigate through multiple maps, cause in some cases, the map is too large to load in rviz.
![Screenshot from 2024-06-08 13-30-25](https://github.com/StanleyChueh/Multi_map_navigation/assets/153347369/ba3b2056-408e-4b4a-90ab-6d97c388b843)

## Usage
### Open terminal1
```
source /opt/ros/foxy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```

### Open terminal2
```
cd turtlebot3_ws
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py 
```

### Open terminal3
```
cd ~
mkdir multi_map_navigation_ws
cd multi_map_navigation_ws
mkdir src
cd src
git clone https://github.com/StanleyChueh/Multi_map_navigation.git
cd ..
colcon build --symlink-install
source install/setup.bash
ros2 launch custom_nav custom_nav.launch.py 
```


 DEMO(Gazebo)
 https://drive.google.com/file/d/1OQNuAbiI8G6znQiOUhtRKtjmuIte6_4B/view?usp=drive_link
