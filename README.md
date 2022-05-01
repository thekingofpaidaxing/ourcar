# ourcar
记录无人车项目开发过程

## Clone this repo
open a terminal window, move to the src folder of your workspace, like 
```
cd ~/catkin_ws/src
```
Clone this repo to your src folder 
```
git clone https://github.com/thekingofpaidaxing/ourcar.git
```
## Create a ROS package
Now create the package 
```
catkin_create_pkg ourcar
cd ~/catkin_ws/
catkin_make
```

## Run a demo
### set up environment
In terminal
```
source ~/catkin_ws/devel/setup.bash
```
### load map and robot
```
rosluanch navigation_stage move_base_amcl_2.5cm.launch
```
### make robot start moving
```
rosrun simple_navigation_goals_tutorial loop_goals
```

