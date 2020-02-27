# Robots Mini-Project

## Cloning
```
git clone git@github.com:KarenSarmiento/RobotMiniProject.git multi_robot
```

## Running
```
source ~/catkin_ws/devel/setup.bash
roslaunch multi_robot gazebo_simple.launch
python obstacle_avoidance.py
```

## Topics
To list the topics being published to:
```
rostopic list
```

## Dependencies
```
sudo apt install ros-kinetic-multirobot-map-merge
```