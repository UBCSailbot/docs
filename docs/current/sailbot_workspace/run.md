# Run Instructions

## Source the ROS workspace overlay

=== "ROS 2"

    ```
    srcnew
    ```

=== "ROS 1"

    ```
    srcraye
    ```

## Build

!!! note ""

    This step is not necessary if no modifications to C++ code or custom msgs were made.

=== "ROS 2"

    Run the "Build" VS Code task, which has the keyboard shortcut ++ctrl+shift+b++.

=== "ROS 1"

    ```
    roscd
    catkin_make
    ```

## Run the ROS program

!!! tip "Running GUI applications"

    For the operating systems that need additional software to run GUI applications like Windows 10 and MacOS,
    ensure that they are running beforehand.

=== "ROS 2"

    `ros2 launch ...` or `ros2 run ...`

=== "ROS 1"

    `roslaunch ...` or `rosrun ...`
