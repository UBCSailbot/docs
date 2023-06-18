# How-To's

## Run optional programs

!!! info ""

    New in [:octicons-tag-24: v1.1.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.1.0){target=_blank}

There are a couple programs that are not run by default to minimize resource usage:

- [Docs site](https://github.com/UBCSailbot/docs){target=_blank}
- [Pathfinding website](https://github.com/UBCSailbot/website){target=_blank}
- [MongoDB database](https://www.mongodb.com/){target=_blank}
- [Grafana dashboards](https://grafana.com/){target=_blank}

To run a program:

1. In `dockerComposeFile` of [`.devcontainer/devcontainer.json`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/devcontainer.json){target=_blank},
   uncomment the Docker Compose file that defines the program
2. Run the `Dev Containers: Rebuild Container` command
3. Follow the program's run instructions:

    ??? note "Run Docs or Website"

        1. Run the `Ports: Focus on Ports View` command
        2. Open the site by hovering over its local address and clicking either "Open in Browser" or "Preview in Editor"
            - Docs is running on port 8000
            - Website is running on port 3005
        3. Changes made to its files are loaded when they are saved

    ??? note "Run MongoDB"

        1. Setup the [MongoDB VS Code extension](https://www.mongodb.com/products/vs-code){target=_blank}
            1. [Connect it to the running database](https://www.mongodb.com/docs/mongodb-vscode/connect/#create-a-connection-to-a-deployment){target=_blank}
                1. Use the default methods: "Paste Connection String" and "Open from Overview Page"
                2. Our database's connection string is `mongodb://localhost:27017`
        2. See the [MongoDB VS Code extension docs](https://www.mongodb.com/docs/mongodb-vscode/){target=_blank} for how
           to use it to navigate or explore the database

    ??? note "Run Grafana"

        1. Connect to the MongoDB database
            0. Prerequisites:
                1. MongoDB is running
                2. Not running Sailbot Workspace in a [GitHub Codespace](./setup.md#setup-sailbot-workspace-in-a-github-codespace){target=_blank}
            1. Open the site by hovering over its local address and clicking "Open in Browser"
                - Grafana is running on port 3000
            2. Login using the default username and password: `admin` and `admin`, respectively
            3. Once logged-in, click the cog icon (:material-cog:) in the bottom left to go to the data sources configuration
               page
            4. Add the `mongodb-community` data source
                1. Paste our database's URL: `mongodb://localhost:27017`
                2. Click "Save & test"
                3. Verify that the message "MongoDB is Responding" pops up

To stop running a program:

1. Stop the program's container (where it is running):
    1. Open Docker Desktop
    2. Select "Containers" in the top right
    3. Expand `sailbot_workspace_devcontainer`
    4. Click the stop icon (:material-stop:) under actions for the container that you want to stop
        - The name should be something like `<program>-1`

    ??? note "Stop the program's container using terminal commands"

        You will have to use this method if you didn't install Docker using Docker desktop.

        In a terminal outside the Dev Container:

        1. Get the program container's name by running `docker ps`, which lists the running containers
            - The container should be named something like `sailbot_workspace_devcontainer-<program>-1`
        2. Stop the program's container by running `docker stop <container>`

2. Stop future rebuilds of the Dev Container from restarting the program's container:
    1. In `dockerComposeFile` of [`.devcontainer/devcontainer.json`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/devcontainer.json){target=_blank},
       comment out the Docker Compose file that defines the program

## Temporarily add apt packages

If a task requires you to add apt packages, you can quickly test them in your Dev Container:

1. Uncomment the section in [`Dockerfile`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/Dockerfile){target=_blank}
   that installs additional packages
2. Add the desired packages below the line `# Your package list here` with the format:

    ```sh
    # Your package list here
    _pkg1_ \
    _pkg2_ \
    ```

3. Run the `Dev Containers: Rebuild Container` command

Before merging in the PR, you should migrate the apt package installations to a more permanent location in upstream images:

- [`base`, `local-base`, `dev`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev){target=_blank}
- [`pre-base`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/pre-base){target=_blank}

## Use your dotfiles

Dotfiles are configuration files for various programs.[^1]

??? info "More about dotfiles"

    - They are called dotfiles because their filenames start with a dot (`.`)
    - On Linux and MacOS, files and directories that begin with a dot are hidden by default
    - To list dotfiles using the `ls` command, specify the `-a` argument: `ls -a`

Dotfiles that are commonly modified include:

- Bash: `~/.bashrc`
- Git: `~/.gitconfig`
- Vim: `~/.vimrc`

To use your dotfiles:

1. Ensure that the [`base`, `local-base`, or `dev` image](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev){target=_blank}
   installs the programs that the dotfiles correspond to
2. Copy the dotfiles to the [`.devcontainer/config/`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/config){target=_blank}
   directory. If a dotfile is located in a child directory, you will have to created it.
   For example, if a dotfile's path is `~/.config/ex_dotfile`, you will need to copy it to `.devcontainer/config/.config/ex_dotfile`

    !!! warning "Special cases"

        - `~/.gitconfig`: there is no need copy your Git dotfile, as Dev Containers do this automatically
        - `~/.bashrc`: don't copy your Bash dotfile, as it would override the one created in the [`dev` image](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/base-dev){target=_blank}.
        Instead, add your bash configuration `.aliases.bash` or `.functions.bash` in the config directory, as these are sourced
        by the created Bash dotfile.

3. Run the `Dev Containers: Rebuild Container` command

[^1]: [Dotfiles – What is a Dotfile and How to Create it in Mac and Linux](https://www.freecodecamp.org/news/dotfiles-what-is-a-dot-file-and-how-to-create-it-in-mac-and-linux/){target=_blank}

## Run Raye's software

Raye was our previous project. Her software can be run in the `raye` branch:

1. Switch to the `raye` branch: `git switch raye`
2. Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` command

!!! warning "`raye` branch disclaimers"

    1. Since `raye` (and Raye's codebase in general) is not in active development, it may not be 100% functional
       or contain all the features in `main`
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

### Raye's known issues

!!! bug "Run commands for Raye packages are very slow"

    On non-Ubuntu-based Linux operating systems, Run commands for Raye packages may take a long time to start-up.
    This is because the system has trouble resolving the local hostname.

    To resolve this bug, run the commands below in the Dev Container:

    ```bash
    echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
    ```
