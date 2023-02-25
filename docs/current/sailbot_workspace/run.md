# Sailbot Workspace Run Instructions

## 1. Rebuild the Dev Container

!!! note "Skip this step if..."

    none of the files in `.devcontainer/` were modified since the Dev Container was last built.

    These file may be modified if you:

    1. `git pull`
    2. Switch branches
    3. Update a file in `.devcontainer/` yourself

    VS Code may prompt you to rebuild the Dev Container when these files are modified,
    but it doesn't do so 100% of the time.

Run the `Dev Containers: Rebuild Container` command in the
[VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette){target=_blank}.

## 2. Run the `setup` VS Code task

??? info "What does the `setup` task do?"

    The `setup` task imports the ROS packages and install their dependencies.

!!! note "Skip this step if..."

    you have run this task before and haven't changed ROS package dependencies since.

To select a task to run, run the `Tasks: Run Task` command in the
[VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette){target=_blank}.

??? bug "Can't see the `setup` task"

    If you can't see the `setup` task, close and reopen VS Code before rerunning the `Tasks: Run Task` command.

## 3. Build the ROS packages

!!! note "Skip this step if..."

    no modifications were made to C++ code, custom msgs, or compiled resources.

Run the `Build` VS Code task, which has the shortcut ++ctrl+shift+b++

??? info "`Build` task shortcut on macOS"

    The `Build` task shortcut on macOS is ++cmd+shift+b++.

## 4. Run the ROS program

??? tip "Running GUI applications on macOS"

    If you want to run GUI applications on macOS, ensure that XQuartz is running.

Run a ROS launch file or package-specific executable with `ros2 launch ...` or `ros2 run ...`, respectively.

## Raye Run Instructions

Raye's software can be run in the `raye` branch:

1. Switch to the `raye` branch: `git switch raye`
2. Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` command in the
   [VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette){target=_blank}

!!! warning "`raye` branch disclaimers`"

    1. Since the `raye` branch (and Raye's codebase in general) is not in active development,
       it may not be 100% functional or contain all the features in `main`
    2. `raye` is more memory intensive than `main` because the parent image of its Dev Container is much larger;
       this may lead to worse performance

### Build Raye's ROS packages

To build Raye's ROS packages, run the following commands:

```bash
roscd
catkin_make
```

### Run packages from different workspaces

The `raye` branch has two ROS workspaces: one for Raye and one for the new project.
To run ROS packages, you will have to source the overlay of the workspace that it is in:

=== "New Project"

    ```
    srcnew
    ```

=== "Raye"

    ```
    srcraye
    ```

Then you can run launch files or package-specific executables in that workspace with:

=== "New Project"

    `ros2 launch ...` or `ros2 run ...`, respectively.

=== "Raye"

    `roslaunch ...` or `rosrun ...`, respectively.

### Raye Known Issues

!!! bug "Run commands for Raye packages are very slow"

    On non-Ubuntu-based Linux operating systems, Run commands for Raye packages may take a long time to start-up.
    This is because the system has trouble resolving the local hostname.

    To resolve this bug, run the commands below in the Dev Container:

    ```bash
    echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
    ```
