# Robot Operating System

<!-- TODO: Convert citations to proper footnotes -->

Robot Operating System (ROS) is a set of software libraries and tools for building robot applications [1].
It provides functionality for hardware abstraction, device drivers, communication between processes over
multiple machines, tools for testing and visualization, and much more [2].

We uses ROS because it is open-source, language-agnostic, and built with cross-collaboration in mind.
It enables our subteams to work independently on well-defined components of our software system,
without having to worry about the hardware it runs on or the implementation of other components.

## Tutorials

[The official ROS 2 documentation](https://docs.ros.org/en/eloquent/index.html) contains everything you need
to get started using it. From it we have hand-picked the tutorials that are most relevant to our software assuming
that you use [our preconfigured workspace](https://github.com/UBCSailbot/sailbot_workspace). Installing, setting up,
and learning ROS without our workspace is outside the scope of this site.

### Sailbot Workspace Configuration

Our workspace configuration contains easier methods of accomplishing some of the tutorial steps, or eliminates the need
for them altogether.

| Tutorial | sailbot_workspace |
| -------- | ----------------- |
| Install a package | Uncomment the relevant section in [`.devcontainer/Dockerfile`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/Dockerfile) then rebuild the dev container to install all the packages used in the tutorials below |
| Source ROS 2 environment | ROS 2 sourced by default in all new terminals |
| Clone a sample repo | Uncomment the "ros_tutorials" section in [`src/new_project.repos`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/new_project.repos) then run the "setup" task |
| Resolve dependencies | Run the "setup" task |
| Build the workspace | Run the "Build" task, or ++ctrl+shift+b++ |
| Source the overlay | `srcnew` |

### Beginner

- [CLI Tools](https://docs.ros.org/en/eloquent/Tutorials.html#beginner-cli-tools)
    - Introducing turtlesim and rqt
    - Understanding ROS 2 nodes
    - Understanding ROS 2 topics
    - Understanding ROS 2 services
    - Understanding ROS 2 parameters
    - Understanding ROS 2 actions
    - Using rqt_console
    - Creating a launch file
    - Recording and playing back data
- [Client Libraries](https://docs.ros.org/en/eloquent/Tutorials.html#beginner-client-libraries)
    - Creating a workspace
    - Creating your first ROS 2 package
    - Writing a simple publisher and subscriber (C++)
    - Writing a simple publisher and subscriber (Python)
    - Writing a simple service and client (C++)
    - Writing a simple service and client (Python)
    - Creating custom ROS 2 msg and srv files
    - Expanding on ROS 2 interfaces
    - Using parameters in a class (C++)
    - Using parameters in a class (Python)
    - Getting started with ros2doctor

### Intermediate

- [General](https://docs.ros.org/en/eloquent/Tutorials.html#intermediate)
    - Creating an action
    - Writing an action server and client (C++)
    - Writing an action server and client (Python)
- [Working With Your First Package & Workspace](https://docs.ros.org/en/eloquent/Tutorials.html#working-with-your-first-package-workspace)
    - Developing a ROS 2 package
    - Using colcon to build packages
    - ament_cmake user documentation
- [Learning the ROS 2 Toolset](https://docs.ros.org/en/eloquent/Tutorials.html#learning-the-ros-2-toolset)
    - Launching/monitoring multiple nodes with Launch
    - Passing ROS arguments to nodes via the command-line
    - Introspection with command line tools
    - Overview and usage of RQt
    - Composing multiple nodes in a single process
    - Overriding QoS Policies For Recording And Playback

### Advanced

- Synchronous vs. asynchronous service clients
- Working with multiple ROS 2 middleware implementations
- On the mixing of ament and catkin (catment)
- Cross-compilation
- Implement a custom memory allocator
- Releasing a ROS 2 package with bloom

### Miscellaneous

- Migrating launch files from ROS 1 to ROS 2
- Eclipse Oxygen with ROS 2 and rviz2 [community-contributed]
- Building ROS 2 on Linux with Eclipse Oxygen [community-contributed]
- Building realtime Linux for ROS 2 [community-contributed]
- Migrating YAML parameter files from ROS 1 to ROS 2

### Demos

- Use quality-of-service settings to handle lossy networks.
- Management of nodes with managed lifecycles.
- Efficient intra-process communication.
- Bridge communication between ROS 1 and ROS 2.
- Recording and playback of topic data with rosbag using the ROS 1 bridge.
- Turtlebot 2 demo using ROS 2.
- TurtleBot 3 demo using ROS 2. [community-contributed]
- MoveIt 2 demo using ROS 2.
- Using tf2 with ROS 2.
- Using URDF with robot_state_publisher.
- Write real-time safe code that uses the ROS 2 APIs.
- Use the robot state publisher to publish joint states and TF.
- Use DDS-Security.
- Logging and logger configuration.

### Examples

- Python and C++ minimal examples.

## Concepts

## ROS Versions

There are two major versions of ROS, aptly named ROS 1 and ROS 2. Our previous project, Raye,
uses ROS 1 because it was the only version available during her design process. Our new project will
use ROS 2, a complete re-design of the framework that tackles the shortcomings of ROS 1 to bring it up
to industry needs and standards [3]. If you are curious about the changes made in ROS 2 compared to 1,
[this article](http://design.ros2.org/articles/changes.html) is a worthwhile read.
Our preconfigured workspace has ROS 2 as well as ROS 1 and Raye's codebase.

ROS 2 contains packages that can be installed alongside ROS 1 to help migrate code from ROS 1 to ROS 2.
As we will be reusing some of Raye's codebase, it is important to have some understanding of these packages.
Here are some relevant pages from the ROS 2 documentation:

## Citations

- [1] <https://docs.ros.org/en/humble/index.html>
- [2] <https://www.toptal.com/robotics/introduction-to-robot-operating-system>
- [3] <https://ubuntu.com/robotics/what-is-ros>
