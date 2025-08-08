# fastbot_waypoints

## Description
This package contains the ROS2 action server that allows a Fastbot to follow waypoints.

0. Set up from git repo:
   ```bash
   cd ~/ros2_ws/src/fastbot_waypoints
   git checkout main

1. Launch the Gazebo simulation:
   ```bash
   cd ~/ros2_ws && colcon build
   source install/setup.bash
   ros2 launch fastbot_gazebo one_fastbot_room.launch.py

2. Launch the Action Server in another terminal:
   ```bash
   cd ~/ros2_ws && source install/setup.bash
   ros2 run fastbot_waypoints fastbot_action_server

3. Run ros tests on default good cases in another terminal:
   ```bash
    cd ~/ros2_ws && source install/setup.bash
    colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
    colcon test-result --all

======
4. Before running ros test on bad case, need to modify the test_waypoint_node.cpp to change 

   the line 16: #define ENABLE_GOOD_TESTS 1
 
   to #define ENABLE_GOOD_TESTS 0

and then rebuild the package in the terminal:
   ```bash
    cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints && source install/setup.bash
    colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
    colcon test-result --all

   - It may be good to relaunch everything from step 1 to 3 above but it is ok to run step 4 only if both the gazebo and action server are still running correctly.

   It will show the error result with timeout as goal can't be reached after 60 seconds goal is sent.
