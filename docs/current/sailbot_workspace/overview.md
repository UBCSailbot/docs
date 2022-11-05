# sailbot_workspace

This repository will get you set up to develop UBCSailbot's software on VS Code. It is based on athackst's
[vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace){target=_blank}.
See their [write-up](https://www.allisonthackston.com/articles/vscode_docker_ros2.html){target=_blank} for a more
in-depth look on how this workspace functions.

## Features

### Style

ROS2-approved formatters are included in the IDE.  

- **C++** uncrustify; config from `ament_uncrustify`
- **Python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/){target=_blank}

### Tasks

There are many pre-defined tasks, see
[our workspace file](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/config/sailbot_workspace.code-workspace){target=_blank}
for a complete listing. Bring up the task menu by typing "Tasks: Run Task" in the command pallete, or creating a keyboard
shortcut for `workbench.action.tasks.runTask`.

### Debugging ([WIP](https://github.com/UBCSailbot/sailbot_workspace/issues/6){target=_blank})

This repository has debug configurations for Python files and Cpp programs. See
[our workspace file](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/config/sailbot_workspace.code-workspace){target=_blank}
for configuration details.
Bring up the debug configurations menu by typing "debug " in the command pallete without the ">" prefix, or select one
from the Run and Debug view.

### Continuous Integration

This repository also has continuous integration that lints and tests our code.
See [`.github/workflows/`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.github/workflows){target=_blank}
for the configuration files.

### Configured Terminal Commands and Aliases

| ROS 2 Command | ROS 1 Command | Function |
| ------------- | ------------- | -------- |
| `srcnew` | `srcraye` | Source ROS workspace overlay |
| `colcon_cd` | `roscd` | Navigate to ROS workspace |
