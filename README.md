# Multi_map_navigation
#### In this project, I aim to navigate through multiple maps, cause in some cases, the map is too large to load in rviz.
![Screenshot from 2024-06-08 13-30-25](https://github.com/StanleyChueh/Multi_map_navigation/assets/153347369/ba3b2056-408e-4b4a-90ab-6d97c388b843)

## Usage 
### Set up config file
📣 Please sepecify the path of your map1,map2,and also modify your initial pose and goal pose if needed. 📣
```
custom_nav:
  ros__parameters:
    map_numbers: 2
    map_0:
      map_url_param: '/home/users/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map
/turtlebot3_world.yaml'
      initial_pose: {x: -2.048, y: -0.561, z: 0.008, yaw: 1.0}
      nav_pose : {x: 1.55 , y: 0.03, z: 0.0, yaw: 1.0}  
    map_1:
      map_url_param: '/home/users/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map
/turtlebot3_world.yaml'
      initial_pose: {x: 1.55 , y: 0.03, z: 0.0, yaw: 1.0} 
      nav_pose : {x: -2.048, y: -0.561, z: 0.008, yaw: 1.0}
    # map_2:
    # map_url_param: '/home/users/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map
/turtlebot3_world.yaml'
    # initial_pose: {x: 155.94, y: 78.79, z: 0.0, yaw: 0.036}
    # nav_pose : {x: 154.115, y: 80.52, z: 0.0, yaw: 0.036}
```

### Open terminal1 💻
```
source /opt/ros/foxy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```

### Open terminal2 💻
```
cd turtlebot3_ws
source install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py 
```

### Open terminal3 💻
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
## Support
🤝 Please create an issue on this GitHub for any questions. This allows other people with the same question to find your answer. 🤝

## DEMO(Gazebo)
https://youtu.be/_x5eRTNBcCU?si=bTipYbon7k8zvZaG
