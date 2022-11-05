## Setup Instructions

This workspace can be set up on most operating systems, but it performs the best and requires the least setup on
Ubuntu and [its derivatives](https://distrowatch.com/search.php?basedon=Ubuntu){target=_blank}.

1. Install prerequisites

    === ":material-microsoft-windows: Windows"

        - Install [WSL](https://learn.microsoft.com/en-us/windows/wsl/about){target=_blank}
            - Run these commands in an *administrator* PowerShell window

                ```
                wsl --install --distribution Ubuntu
                wsl --set-default Ubuntu
                wsl --set-version Ubuntu 2
                ```

        - [Docker](https://docs.docker.com/get-started/overview/){target=_blank}
            - [Install Docker Desktop](https://docs.docker.com/engine/install/){target=_blank}
            - Use the WSL 2 backend for Docker Desktop

    === ":material-apple: MacOS"

        - [Docker](https://docs.docker.com/get-started/overview/){target=_blank}
            - [Install Docker Desktop](https://docs.docker.com/engine/install/){target=_blank}

    === ":material-linux: Linux"

        - [Docker](https://docs.docker.com/get-started/overview/){target=_blank}
            - [Install Docker Engine](https://docs.docker.com/engine/install/){target=_blank}
            - [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user){target=_blank}
            - [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot){target=_blank}

    - [VS Code](https://code.visualstudio.com/){target=_blank}
        - [Install VS Code](https://code.visualstudio.com/download){target=_blank}
        - [Install VS Code Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack){target=_blank}

2. For Windows and MacOS, additional configuration to run GUI applications

    === ":simple-windows11: Windows 11"

        GUI applications work without additional configuration,
        *but if you upgraded from Windows 10 make sure to update the WSL kernel:* `wsl --update`
        in an *administrator* PowerShell window

    === ":material-microsoft-windows: Windows 10"

        Follow [this guide](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242){target=_blank}
        up to, but not including setting the `DISPLAY` variable. Then, add the following to your `~/.bashrc` of the WSL
        filesystem:

        ```bash
        export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
        ```

        If VS Code is open, then restart it.

    === ":material-apple: MacOS"

        ### XQuartz Configuration
        Follow [this guide](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088){target=_blank} to setup 
        XQuartz. Then, follow these additional XQuartz configuration steps:

        ```bash
        # Copy contents to ~/.xinitrc
        cp /opt/X11/etc/X11/xinit/xinitrc ~/.xinitrc

        # Append "xhost +localhost" to .xinitrc
        echo "xhost +localhost" >> ~/.xinitrc
        ```

        If XQuartz is open, then restart it.

        ### Zsh Configuration

        Execute these commands in your terminal:

        ```bash
        echo 'export MAC_DOCKER_LOCALHOST="docker.for.mac.host.internal"' >> ~/.zshrc
        echo 'export DISPLAY=:0' >> ~/.zshrc
        ```
        If VS Code is open, then restart it.

    ??? note "Arch Linux"

        - Run These Commands Locally to setup X11 fowarding 
            - Install xhost using `sudo pacman -S xorg-xhost`
            - `export DISPLAY=0.0`
            - `xhost +local:docker`
        - Dev-Containers only works on the right flavor of Vscode, make sure you are using the official Microsoft distribution
            - The one from Pacman doesn't work with microsoft plugins
        - After the image is running on your machine, follow standard procedures

    ??? bug "Linux Debugging"

        To run Ros1, run these commands as the system has trouble resolving the local hostname  
            ```
            export ROS_HOSTNAME=localhost
            export ROS_MASTER_URI=http://localhost:11311
            ```

3. Clone the Sailbot Workspace repository

    ```
    git clone https://github.com/UBCSailbot/sailbot_workspace.git
    ```

    - For Windows, clone the repository in the WSL filesystem, for example `~/sailbot` in the Ubuntu WSL terminal

4. Open it in VS Code

    ```
    code sailbot_workspace
    ```

5. Open it in a container:
    1. Make sure that Docker is running
    2. When you open it for the first time, you should see a little popup that asks you if you would like to open it in
       a container. Say yes!
    3. If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up
       the container dialog
        - In the dialog, select "Reopen in container"
    4. VSCode will build the dockerfile inside of `.devcontainer` for you. If you open a terminal inside VSCode
       (Terminal > New Terminal), you should see that your username has been changed to `ros`, and the bottom left green
       corner should say "Dev Container"

6. Open the workspace file `.devcontainer/config/sailbot_workspace.code-workspace` and click "Open Workspace"

7. Import the ROS packages and install their dependencies by running the "setup" task
