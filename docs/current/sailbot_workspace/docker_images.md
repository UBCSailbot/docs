# Sailbot Workspace Docker Images

A table detailing the Docker images used to create the Dev Container can be found below.
Click on an image to learn more about its features and how to update it.

| Image                                         | Parent Image                                | Source Code                                                 | Why it is Rebuilt                | Where it is Built |
| --------------------------------------------- | ------------------------------------------- | ----------------------------------------------------------- | ---------------------------------| ----------------- |
| [`pre-base`][pre-base]{target=_blank}         | [Ubuntu 22.04][Ubuntu Image]{target=_blank} | [`pre-base.Dockerfile`][pre-base.Dockerfile]{target=_blank} | To install ROS or OMPL           | Personal computer |
| [`base`][base]{target=_blank}                 | [`pre-base`][pre-base]{target=_blank}       | [`base-dev.Dockerfile`][base-dev.Dockerfile]{target=_blank} | To install core dependencies     | Workflow dispatch |
| [`local-base`][local-base]{target=_blank}     | [`base`][base]{target=_blank}               | [`base-dev.Dockerfile`][base-dev.Dockerfile]{target=_blank} | To install core dev dependencies | Workflow dispatch |
| [`dev`][dev]{target=_blank}                   | [`local-base`][local-base]{target=_blank}   | [`base-dev.Dockerfile`][base-dev.Dockerfile]{target=_blank} | To install dev dependencies      | Workflow dispatch |
| [Dev Container][Dev Container]{target=_blank} | [`dev`][dev]{target=_blank}                 | [`Dockerfile`][Dockerfile]{target=_blank}                   | To configure the Dev Container   | VS Code           |

<!-- Images URLs -->
[Ubuntu image]: <https://hub.docker.com/_/ubuntu>
[pre-base]: <https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/pre-base>
[base]: <https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev>
[local-base]: <https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev>
[dev]: <https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev>
[Dev Container]: <https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer>

<!-- Dockerfile URLs -->
[pre-base.Dockerfile]: <https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/pre-base/pre-base.Dockerfile>
[base-dev.Dockerfile]: <https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/base-dev/base-dev.Dockerfile>
[Dockerfile]: <https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/Dockerfile>
