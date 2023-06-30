# How-To's

## Run optional programs

!!! info ""

    New in [:octicons-tag-24: v1.1.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.1.0){target=_blank}

There are a couple programs that are not run by default to minimize resource usage:

- [Docs site](https://github.com/UBCSailbot/docs){target=_blank}
- [Pathfinding website](https://github.com/UBCSailbot/website){target=_blank}
- [MongoDB database](https://www.mongodb.com/){target=_blank}
- [Grafana dashboards](https://grafana.com/){target=_blank}

### Start running an optional program

1. In the first section of `dockerComposeFile` of `.devcontainer/devcontainer.json`,
   there is a list of Docker Compose files with comments at the end of the line that say which programs are defined in them.
   Uncomment the Docker Compose file that the program is defined in: remove `//` at the beginning of the line
    1. Programs that are defined in the uncommented Docker Compose files will be started and stopped with Sailbot Workspace
2. Run the `Dev Containers: Rebuild Container` VS Code command to restart Sailbot Workspace
3. Follow the program-specific run instructions:

    ??? note "Run Docs or Website"

        1. Run the `Ports: Focus on Ports View` VS Code command
        2. Open the site by hovering over its local address and clicking either "Open in Browser" or "Preview in Editor"
            - The local address of Docs is the line with a port of 8000
            - The local address of Website is the line with a port of 3005
        3. Changes made to its files are loaded when they are saved
            1. **If Auto Save is on, turn it off so that the Docs/Website servers aren't continuously reloading**
                - Auto Save is on by default in GitHub Codespaces

    ??? note "Run MongoDB"

        1. Setup the [MongoDB VS Code extension](https://www.mongodb.com/products/vs-code){target=_blank}
            1. [Connect it to the running database](https://www.mongodb.com/docs/mongodb-vscode/connect/#create-a-connection-to-a-deployment){target=_blank}
                1. Use the default methods: "Paste Connection String" and "Open from Overview Page"
                2. Our database's connection string is `mongodb://localhost:27017`
        2. See the [MongoDB VS Code extension docs](https://www.mongodb.com/docs/mongodb-vscode/){target=_blank} for how
           to use it to navigate or explore the database

    ??? note "Run Grafana"

        1. Connect to the MongoDB database
            1. Ensure that:
                1. You are not running Sailbot Workspace in a [GitHub Codespace](./setup.md#setup-sailbot-workspace-in-a-github-codespace){target=_blank}
                2. MongoDB is running
            2. Open the site by hovering over its local address and clicking "Open in Browser"
                - The local address of Grafana is the line with a port of 3000
            3. Login using the default username and password: `admin` and `admin`, respectively
            4. Once logged-in, click the cog icon :material-cog: in the bottom left to go to the data sources configuration
               page
            5. Add the `mongodb-community` data source
                1. Paste our database's URL: `mongodb://localhost:27017`
                2. Click "Save & test"
                3. Verify that the message "MongoDB is Responding" pops up

### Manage an optional program

Each program runs in a Docker container. Containers can be managed using Docker Desktop or CLI commands:

- View Sailbot Workspace containers

    === ":material-docker: Docker Desktop"

        1. Select "Containers" in the top right
        2. Expand "sailbot_workspace_devcontainer"
            - The "Status" column shows whether a container is running or not

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker ps -a
        ```

        - Sailbot Workspace containers should be named something like `sailbot_workspace_devcontainer-<program>-<number>`
        - The `STATUS` column shows whether a container is running or not

- View a container's logs, the output of the container (including errors that caused it to stop)

    === ":material-docker: Docker Desktop"

        1. Click on a container
        2. Navigate to the "Logs" view if not already on it

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker logs <container>
        ```

- Start a container that is not running

    === ":material-docker: Docker Desktop"

        1. Click start :material-play:

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker start <container>
        ```

- Stop a container that is running

    === ":material-docker: Docker Desktop"

        1. Click stop :material-stop:

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker stop <container>
        ```

### Stop running an optional program

1. In `dockerComposeFile` of `.devcontainer/devcontainer.json`, comment out the Docker Compose file that the program is
   defined in: add `//` to the beginning of the line
2. Stop the program's container: see [Manage an optional program](#manage-an-optional-program)

## Temporarily add apt packages

If a task requires you to add apt packages, you can quickly test them in your Dev Container:

1. Uncomment the section in `Dockerfile` that installs additional packages
2. Add the desired packages below the line `# Your package list here` with the format:

    ```sh
    # Your package list here
    _pkg1_ \
    _pkg2_ \
    ```

3. Run the `Dev Containers: Rebuild Container` VS Code command

Before merging in the PR, you should migrate the apt package installations to a more permanent location in upstream
[images](./docker_images.md){target=_blank}: `base`, `local-base`, `dev`, or `pre-base`.

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

1. Ensure that the `base`, `local-base`, or `dev` [image](./docker_images.md){target=_blank}
   installs the programs that the dotfiles correspond to
2. Copy the dotfiles to the `.devcontainer/config/` directory.
   If a dotfile is located in a child directory, you will have to created it.
   For example, if a dotfile's path is `~/.config/ex_dotfile`, you will need to copy it to `.devcontainer/config/.config/ex_dotfile`

    !!! warning "Special cases"

        - `~/.gitconfig`: there is no need copy your Git dotfile, as Dev Containers do this automatically
        - `~/.bashrc`: don't copy your Bash dotfile, as it would override the one created in the `dev` image.
        Instead, add your bash configuration `.aliases.bash` or `.functions.bash` in the config directory, as these are sourced
        by the created Bash dotfile.

3. Run the `Dev Containers: Rebuild Container` VS Code command

[^1]: [Dotfiles – What is a Dotfile and How to Create it in Mac and Linux](https://www.freecodecamp.org/news/dotfiles-what-is-a-dot-file-and-how-to-create-it-in-mac-and-linux/){target=_blank}

## Run Raye's software

Raye was our previous project. Her software can be run in the `raye` branch:

1. Switch to the `raye` branch: `git switch raye`
2. Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` VS Code command

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
