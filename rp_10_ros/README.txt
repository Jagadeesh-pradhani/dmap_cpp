0. initialize a workspace called dmap_exercise_workspace

1  inside this workspace, create a package_called dmap_live_registration
   that has the following dependencies
   - roscpp
   - sensor_msgs
   - cv_bridge (for visualization and to link opencv)

2. copy *the folder* rp_stuff in the folder src of the newly created package

3. edit the top level CMakeLists.txt of the package and of rp_stuff to build the files in the folder with catkin build. We provided a CMakeLists_standalone.txt to ensure the files can be built in a standalone fashion

4. prepare the testing environment by starting
   roscore
   rosrun stage_ros stageros cappero***.world
   
5. add the the program dmap_show_live_node.cpp to the build of the package
   and address the todoes
   
   start it to listen the /base/scan topic
   
6. add the the dmap_localize_node.cpp to the build of the package
   and address the todoes
   enjoy moving (slowly) the robot on satge

7. add the the dmap_tracker_node.cpp to the build of the package
   dmap_tracker_node.cpp and address the todoes
   enjoy moving (slowly) the robot on satge




