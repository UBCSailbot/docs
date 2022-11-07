# Setup Instructions

This workspace can be set up on most operating systems, but it performs the best and requires the least setup on
Ubuntu and [its derivatives](https://distrowatch.com/search.php?basedon=Ubuntu){target=_blank}.

## Install and configure prerequisites

### Docker

=== ":material-microsoft-windows: Windows"

    1. [Install Docker Desktop](https://docs.docker.com/desktop/install/windows-install/){target=_blank}
    with the WSL 2 backend

    2. Configure [WSL](https://learn.microsoft.com/en-us/windows/wsl/about){target=_blank}
        1. Run the commands below in an **administrator** PowerShell window

            ```powershell
            wsl --install --distribution Ubuntu
            wsl --set-default Ubuntu
            wsl --set-version Ubuntu 2
            ```

        2. Run `whoami` in the Ubuntu terminal; if it outputs `root`, create a non-root user with sudo privileges
        and give it full permissions to Docker
            1. [Create a non-root user with sudo privileges](https://www.digitalocean.com/community/tutorials/how-to-add-and-delete-users-on-ubuntu-20-04){target=_blank}
            2. Change the default Ubuntu user to the one you just created by entering the following command in Powershell:
            `ubuntu config --default-user <username>`, where `<username>` is the name of the user
            3. [Add the user to the Docker group](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user){target=_blank}

=== ":material-apple: MacOS"

    1. [Install Docker Desktop](https://docs.docker.com/engine/install/){target=_blank}

=== ":material-linux: Linux"

    1. [Install Docker Engine](https://docs.docker.com/engine/install/){target=_blank}
    2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user){target=_blank}
    3. [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot){target=_blank}

### VS Code

- [VS Code](https://code.visualstudio.com/){target=_blank}
    1. [Install VS Code](https://code.visualstudio.com/download){target=_blank}
        - For Arch Linux, make sure install the official Microsoft distribution;
        the one from Pacman doesn't work with Microsoft plugins
    2. [Install VS Code Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack){target=_blank}

## Additional configuration to run GUI applications

=== ":material-microsoft-windows: Windows"

    === ":simple-windows11: Windows 11"

        GUI applications work without additional configuration,
        **but if you upgraded from Windows 10 make sure to update the WSL kernel:** `wsl --update`
        in an **administrator** PowerShell window

    === ":material-microsoft-windows: Windows 10"

        1. Follow [this guide](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242){target=_blank}
        up to, but not including setting the `DISPLAY` variable
        2. Zsh configuration
            1. Run the command below in your Ubuntu terminal

                ```bash
                echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
                echo 'export WIN10_DOCKER_DISPLAY_END="${DISPLAY:1}"' >> ~/.bashrc
                ```

        3. If VS Code is open, then restart it

=== ":material-apple: MacOS"

    1. XQuartz configuration

        1. Follow [this guide](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088){target=_blank} to setup 
        XQuartz
        2. Copy the default xinitrc to your home directory

            ```zsh
            cp /opt/X11/etc/X11/xinit/xinitrc ~/.xinitrc
            ```

        3. Add `xhost +localhost` to `~/.xinitrc` after its first line
        4. If XQuartz is open, then restart it

    2. Zsh configuration

        1. Run the commands below in your MacOS terminal

            ```zsh
            echo 'export MAC_DOCKER_LOCALHOST="docker.for.mac.host.internal"' >> ~/.zshrc
            echo 'export DISPLAY=:0' >> ~/.zshrc
            ```

        2. If VS Code is open, then restart it

=== ":material-linux: Linux"

    === ":material-ubuntu: Ubuntu and its derivatives"

        GUI applications should work without additional configuration.

    === ":material-linux: Other"

        === ":material-arch: Arch Linux"

            1. Set up X11 forwarding

                1. Install xhost

                    ```bash
                    sudo pacman -S xorg-xhost
                    ```

                2. Run the commands below in your Linux terminal

                    ```bash
                    echo 'export DISPLAY=0.0' >> ~/.bashrc
                    cp /etc/X11/xinit/xinitrc ~/.xinitrc
                    ```

                3. Add `xhost +local:docker` to `~/.xinitrc` after its first line

        ??? bug "ROS 1 not working"

            The system has trouble resolving the local hostname; run the commands below in your VS Code terminal

            ```bash
            echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
            echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
            ```

## Clone sailbot_workspace

!!! notes "Note for Windows"

    Run the command below in the Ubuntu terminal.

```sh
git clone https://github.com/UBCSailbot/sailbot_workspace.git
```

## Open sailbot_workspace in VS Code

```sh
code sailbot_workspace
```

## Open sailbot_workspace in a Dev Container

1. Make sure that Docker is running
2. Run the "Dev Containers: Reopen in Container" command in the VS Code command pallete

## Open the sailbot_workspace VS Code workspace

1. Open the file `.devcontainer/config/sailbot_workspace.code-workspace` in VS Code
2. Click "Open Workspace"

## Run the VS Code task named "setup"

This imports the ROS packages and install their dependencies.

## Updating sailbot_workspace

When changes to the Dev container are made (any file in `.devcontainer/`), it needs to be rebuilt.
This may happen when you pull the latest commits from a branch or switch branches.

1. Run the "Dev Containers: Rebuild Container" command in the VS Code command palette
