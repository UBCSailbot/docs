# Setup Instructions

This workspace can be run on Windows, Linux, or macOS, but is the easiest to set up and performs the best on
[Ubuntu](https://ubuntu.com/){target=_blank} and [its derivatives](https://distrowatch.com/search.php?basedon=Ubuntu){target=_blank}.

## Install and configure prerequisites

### Docker

!!! info "Docker Desktop vs Docker Engine"

    Docker Engine is what's commonly known as [Docker](https://www.docker.com/){target=_blank},
    a platform that uses OS-level virtualization to deliver software in packages called containers.[^1]
    Docker Engine can only be installed on Linux. Docker Desktop stores containers and images in a VM[^2],
    allowing it to be installed on Windows and macOS in addition to Linux.

    [^1]: [Wikipedia Docker page](https://en.wikipedia.org/wiki/Docker_(software)){target=_blank}
    [^2]: [What is the difference between Docker Desktop for Linux and Docker Engine](https://docs.docker.com/desktop/faqs/linuxfaqs/#what-is-the-difference-between-docker-desktop-for-linux-and-docker-engine){target=_blank}

=== ":material-microsoft-windows: Windows"

    1. Install prerequisites, [WSL](https://learn.microsoft.com/en-us/windows/wsl/){target=_blank} and [Ubuntu](https://ubuntu.com/){target=_blank}:

        1. In PowerShell, run `wsl --install Ubuntu`

            ??? warning "Already installed WSL or Ubuntu?"

                If WSL or Ubuntu is already installed, ensure that they are up-to-date:

                1. Update WSL by running `wsl --update` in PowerShell
                2. Ensure that Ubuntu is WSL 2, not 1:
                    1. Check the WSL versions of Linux distributions with `wsl -l -v`
                    2. If Ubuntu's `VERSION` is 1, upgrade it to WSL 2 with `wsl --set-version Ubuntu 2`

        2. Open Ubuntu to set up or verify its configuration:
            1. If you are opening Ubuntu for the first time, a setup process will run;
            follow the prompts to finish setting it up
            2. Run `whoami` to verify that it returns your Ubuntu username

                ??? bug "`whoami` returns `root`"

                    If `whoami` returns `root`:

                    1. [Create a non-root user with sudo privileges](https://www.digitalocean.com/community/tutorials/how-to-add-and-delete-users-on-ubuntu-20-04){target=_blank}
                    2. Change the default Ubuntu user to this newly-created user: run `ubuntu config --default-user <username>`
                    in PowerShell, replacing `<username>` with the name of the newly-created user
                    3. Run `whoami` after closing and reopening Ubuntu to verify that it returns your Ubuntu username

    2. [Install Docker Desktop](https://docs.docker.com/desktop/install/windows-install/){target=_blank}
    with the WSL 2 backend

=== ":material-apple: macOS"

    1. [Install Docker Desktop](https://docs.docker.com/desktop/install/mac-install/){target=_blank} for the CPU
       your computer has

=== ":material-linux: Linux"

    1. [Install Docker Engine](https://docs.docker.com/engine/install/){target=_blank}

        ??? warning "Docker Desktop for Linux"

            > Last updated February 2023

            Sailbot Workspace (more specifically its use of VS Code Dev Containers) isn't compatible with Docker Desktop
            for Linux. If you have Docker Desktop installed, uninstall it and install Docker Engine instead.

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
        2. Bash configuration
            1. Run the command below in your Ubuntu terminal

                ```bash
                echo "export DISPLAY=\"`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0\"" >> ~/.bashrc
                echo 'export WIN10_DOCKER_DISPLAY_END="${DISPLAY:1}"' >> ~/.bashrc
                ```

        3. If VS Code is open, then restart it

=== ":material-apple: macOS"

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

        1. Run the commands below in your macOS terminal

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

    Run the command below in the Ubuntu terminal to clone sailbot_workspace in the WSL filesystem.

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
