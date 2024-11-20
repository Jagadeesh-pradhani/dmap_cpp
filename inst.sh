# Dependecies
sudo apt install xterm


# Clone
cd
git clone https://github.com/Jagadeesh-pradhani/dmap_cpp.git

# Build
cd
mkdir -p dmap_ws/src
cp -r ~/dmap_cpp/dmap_live_registration/ ~/dmap_ws/src/
cd ~/dmap_ws/
rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src -r
catkin_make
source ~/dmap_ws/devel/setup.bash

# Commands
rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world
rosrun map_server map_server map.yaml
rosrun tf static_transform_publisher -4 -13 0 0 0 0 1 odom map 10

rosrun dmap_live_registration dmap_localize_node /base_scan

#launch
roslaunch dmap_live_registration dmap.launch node:=dmap_show_live_node
