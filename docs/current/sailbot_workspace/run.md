# Run Instructions

1. For Windows 10 and MacOS, if you want to run something with a GUI
    - For Windows 10, open the XLaunch configuration file
    - For MacOS, start XQuartz

2. Source the relevant overlay in the terminal
    - ROS 2: `srcnew`
    - ROS 1: `srcraye`

3. Build (this step might not be necessary if there are no changes made to C++ or custom msg nodes)
    - ROS 2: run the "Build" VS Code task, which has the keyboard shortcut `CTRL+SHIFT+B`
    - ROS 1: `roscd` then `catkin_make`

4. Run the ROS program
    - ROS 2: `ros2 run ...` or `ros2 launch ...`
    - ROS 1: `rosrun ...` or `roslaunch ...`
