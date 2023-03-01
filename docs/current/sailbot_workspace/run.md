# Sailbot Workspace Run Instructions

## 1. Rebuild the Dev Container

You can skip this step if none of the files in `.devcontainer/` were modified since the Dev Container was last built.

These file may be modified if you:

1. `git pull`
2. Switch branches
3. Update a file in `.devcontainer/` yourself

VS Code may prompt you to rebuild the Dev Container when these files are modified, but it doesn't do so 100% of the time.

1. Run the `Dev Containers: Rebuild Container` command in the
   [VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette){target=_blank}

## 2. Run the `setup` VS Code task

The `setup` task imports the ROS packages and install their dependencies. You can skip this step if you ran this
task before and haven't changed ROS package dependencies since.

1. Run the `Tasks: Run Task` command in the
   [VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette){target=_blank}
2. Select `setup` from the dropdown menu

    ??? bug "Can't see the `setup` task"

        If you can't see the `setup` task, close and reopen VS Code before rerunning the `Tasks: Run Task` command.
        This may occur when Sailbot Workspace is opened for the first time.

## 3. Build the ROS packages

You can skip this step if no modifications were made to C++ code, custom msgs, or compiled resources.

1. Run the `Build` VS Code task: ++ctrl+shift+b++
    - On macOS the shortcut is ++cmd+shift+b++

## 4. Run the ROS program

Run a ROS launch file or package-specific executable with `ros2 launch ...` or `ros2 run ...`, respectively.

??? tip "Running GUI applications on macOS"

    If you want to run GUI applications on macOS, ensure that XQuartz is running.

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
