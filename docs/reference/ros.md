# Robot Operating System

Robot Operating System (ROS) is a set of software libraries and tools for building robot applications.[^1]
It provides functionality for hardware abstraction, device drivers, communication between processes over
multiple machines, tools for testing and visualization, and much more.[^2]

We use ROS because it is open-source, language-agnostic, and built with cross-collaboration in mind.
It enables our subteams to work independently on well-defined components of our software system
without having to worry about the hardware it runs on or the implementation of other components.

[The official ROS 2 documentation](https://docs.ros.org/en/eloquent/index.html) contains everything you need
to get started using ROS. From it we have hand-picked the resources that are most relevant to our current and expected
future usage of ROS assuming that you use [our preconfigured workspace](https://github.com/UBCSailbot/sailbot_workspace).
To run our software on your device without our workspace, you would have to [install ROS](https://docs.ros.org/en/eloquent/Installation.html)
and the dependencies that are in [our Docker images](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer)
yourself.

## Workspace Configuration

To get our workspace configuration running on your computer:

1. Set it up by following the [setup instructions](https://github.com/UBCSailbot/sailbot_workspace#setup)
2. Uncomment the ROS 2 tutorials section in [`.devcontainer/Dockerfile`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/Dockerfile),
   then run the "Remote-Containers: Rebuild Container" VS Code command, to install the tutorials' dependencies
3. Uncomment the ROS 2 tutorials section in [`src/new_project.repos`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/new_project.repos),
   then run the "setup" VS Code task, to clone the repositories used in the tutorials

Our workspace configuration contains easier methods of accomplishing some of the tutorial steps, or eliminates the need
for them altogether.

| Tutorial | sailbot_workspace |
| -------- | ----------------- |
| Install a package | All packages used in the tutorials are already installed (step 2 above) |
| Clone a sample repo (ros_tutorials) | ros_tutorials is already cloned (step 3 above) |
| Resolve dependencies | Run the "install dependencies" VS Code task |
| Build the workspace | Run the "Build" VS Code task, AKA ++ctrl+shift+b++ |
| Source the overlay | Run the `srcnew` terminal command |
| Create a package with a node | Run the "new ament_(python\|cmake) package with a node" VS Code task |

## Tutorials

We encourage all software members to work through the tutorials listed below in order.
For tutorials that have both C++ and Python versions, NET members should do the C++ version
while CTRL and PATH members should do the Python version.

[CLI Tools](https://docs.ros.org/en/eloquent/Tutorials.html#beginner-cli-tools)

- Introducing turtlesim and rqt
- Understanding ROS 2 nodes
- Understanding ROS 2 topics
- Understanding ROS 2 services
- Understanding ROS 2 parameters
- Understanding ROS 2 actions
- Using rqt_console
- Recording and playing back data

[Client Libraries](https://docs.ros.org/en/eloquent/Tutorials.html#beginner-client-libraries)

- Creating a workspace
- Creating your first ROS 2 package
- Writing a simple publisher and subscriber (C++ or Python)
- Writing a simple service and client (C++ or Python)
- Using parameters in a class (C++ or Python)
- Getting started with ros2doctor

## Concepts

We encourage all software members to read the following documentation on key ROS concepts:

- [About logging and logger configuration](https://docs.ros.org/en/eloquent/Concepts/Logging.html)

## ROS 1 Bridge

There are two major versions of ROS, aptly named ROS 1 and ROS 2. Our previous project, Raye,
uses ROS 1 because it was the only version available during her design process. Our new project will
use ROS 2, a complete re-design of the framework that tackles the shortcomings of ROS 1 to bring it up
to industry needs and standards.[^3] If you are curious about the changes made in ROS 2 compared to 1,
[this article](http://design.ros2.org/articles/changes.html) is a worthwhile read.

ROS 2 includes the ROS 1 Bridge, a collection of packages that can be installed alongside ROS 1 to help migrate code
from ROS 1 to ROS 2. As we will be reusing parts of Raye's codebase, it is essential to know how to use these packages.
Until we are completely done with Raye, our preconfigured workspace will have ROS 1, ROS 1 Bridge, and ROS 2 installed.

We encourage all software members work through the [ROS 1 Bridge README](https://github.com/ros2/ros1_bridge/blob/master/README.md).
For PATH members, the [Migrating launch files from ROS 1 to ROS 2 page](https://docs.ros.org/en/eloquent/Tutorials/Launch-files-migration-guide.html)
will be a helpful reference when we do so.

[^1]: <https://docs.ros.org/en/humble/index.html>
[^2]: <https://www.toptal.com/robotics/introduction-to-robot-operating-system>
[^3]: <https://ubuntu.com/robotics/what-is-ros>
