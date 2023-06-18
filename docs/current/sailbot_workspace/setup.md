# Setup Instructions

This workspace can be run on Windows, Linux, or macOS, but is the easiest to set up and performs the best on
[Ubuntu](https://ubuntu.com/){target=_blank} and [its derivatives](https://distrowatch.com/search.php?basedon=Ubuntu){target=_blank}.

Throughout this documentation, there are references to running commands, tasks, and launch configurations in VS Code.
Expand the box below to learn how to do so:

??? note "Running commands, tasks, and launch configurations in VS Code"

    > For keyboard shortcuts on MacOS, substitute ++ctrl++ with ++cmd++

    Commands can be run in the Command Palette.
    Open the Command Palette from the `View` menu or with ++ctrl+shift+p++.

    Tasks can be run using the `Tasks: Run Task` command. Build tasks can be run with ++ctrl+shift+b++.

    Launch configurations can be run from the [Run and Debug view](https://code.visualstudio.com/docs/editor/debugging#_run-and-debug-view){target=_blank}.

    You can also run commands, tasks, launch configurations, and much more by typing their prefixes
    into an empty Command Palette. Open an empty Command Palette with ++ctrl+p++.
    See the list below for some prefixes and their functions.
    For prefixes that are words, you will have to append a space to them to bring up their functions.

    - Nothing: files
    - `>`: commands
    - `task`: tasks
    - `debug`: launch configurations
    - `?`: list all prefixes and their functions

## 1. Set up prerequisites

### Docker

[Docker](https://www.docker.com/){target=_blank} is a platform that uses OS-level virtualization[^1] to develop, ship, and
run applications.[^2] We use it to separate our applications from our infrastructure[^2] so that we can update and
version control our infrastructure for every use case (software members, CI, deployment) in one place: this repository.

[Docker Engine](https://docs.docker.com/engine/){target=_blank} is a software used to run Docker.
However, it can only be installed on Linux.
[Docker Desktop](https://docs.docker.com/desktop/){target=_blank} is a software used to run Docker in a VM,[^3]
allowing it to be installed on Windows and macOS in addition to Linux.

[^1]: [Wikipedia Docker page](https://en.wikipedia.org/wiki/Docker_(software)){target=_blank}
[^2]: [Get Docker](https://docs.docker.com/get-docker/){target=_blank}
[^3]: [What is the difference between Docker Desktop for Linux and Docker Engine](https://docs.docker.com/desktop/faqs/linuxfaqs/#what-is-the-difference-between-docker-desktop-for-linux-and-docker-engine){target=_blank}

=== ":material-microsoft-windows: Windows"

    1. Set up prerequisites, [WSL](https://learn.microsoft.com/en-us/windows/wsl/){target=_blank} and [Ubuntu](https://ubuntu.com/){target=_blank}:

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
                    3. Run `whoami` after closing and reopening Ubuntu, verifying that it returns your Ubuntu username

    2. [Install Docker Desktop](https://docs.docker.com/desktop/install/windows-install/){target=_blank}
    with the WSL 2 backend

=== ":material-apple: macOS"

    [Install Docker Desktop](https://docs.docker.com/desktop/install/mac-install/){target=_blank} for your computer's CPU.

=== ":material-linux: Linux"

    1. [Install Docker Engine](https://docs.docker.com/engine/install/){target=_blank}
        - As of February 2023, Sailbot Workspace (more specifically its use of VS Code Dev Containers) isn't compatible
          with Docker Desktop for Linux; if you have Docker Desktop installed, uninstall it and install Docker Engine instead.
    2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user){target=_blank}
    3. [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot){target=_blank}

### VS Code

[Visual Studio Code](https://code.visualstudio.com/){target=_blank} is a powerful and customizable code editor for
Windows, Linux, and macOS. We strongly recommend that you use this editor to develop our software so that you can
use all the features of Sailbot Workspace.

1. [Install VS Code](https://code.visualstudio.com/docs/setup/setup-overview){target=_blank}
2. Install the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack){target=_blank}

## 2. Set up X11 forwarding

X11 forwarding is a mechanism that enables Sailbot Workspace to run GUI applications. You can skip this step if
you aren't running any GUI applications.

1. Ensure that the versions of VS Code and its Dev Containers extension support X11 forwarding:
    1. VS Code version >= 1.75
    2. Dev Containers version >= 0.275.1
2. Verify that `echo $DISPLAY` returns something like `:0`

    ??? warning "`echo $DISPLAY` doesn't return anything"

        If `echo $DISPLAY` doesn't return anything, set it to `:0` on shell initialization:

        1. Find out what shell you are using with `echo $SHELL`
            1. Most Linux distributions use Bash by default, whose rc file path is `~/.bashrc`
            2. macOS uses Zsh by default, whose rc file path is: `~/.zshrc`
        2. Run `echo 'export DISPLAY=:0' >> <rc file path>`, replacing `<rc file path>`
           with the path to your shell's rc file
        3. Run `echo $DISPLAY` after closing and reopening your terminal, verifying it returns something like `:0`

3. Install a X11 server

    === ":material-microsoft-windows: Windows"

        WSL includes a X11 server.

    === ":material-apple: macOS"

        1. Set up XQuartz following [this guide](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088){target=_blank}
        2. Copy the default xinitrc to your home directory: `cp /opt/X11/etc/X11/xinit/xinitrc ~/.xinitrc`
        3. Add `xhost +localhost` to `~/.xinitrc` after its first line

    === ":material-linux: Linux"

        === ":material-linux: General"

            As of February 2023, almost all Linux distributions include a X11 server, Xorg.
            This may change in the future as Wayland matures.

        === ":material-arch: Arch Linux"

            1. Install xhost: `sudo pacman -S xorg-xhost`
            2. Copy the default xinitrc to your home directory: `cp /etc/X11/xinit/xinitrc ~/.xinitrc`
            3. Add `xhost +local:docker` to `~/.xinitrc` after its first line

4. Verify that X11 forwarding works:
    1. Install `x11-apps`

        === ":material-microsoft-windows: Windows"

            In Ubuntu, `sudo apt install x11-apps`.

        === ":material-apple: macOS"

            XQuartz includes `x11-apps`. Ensure that XQuartz is running.

        === ":material-linux: Linux"

            Install `x11-apps` using your desired package manager.

    2. Verify that running `xcalc` opens a calculator and that you can use it

## 3. Clone Sailbot Workspace

```sh
git clone https://github.com/UBCSailbot/sailbot_workspace.git
```

??? tip "Where to clone on Windows"

    Windows has a native file system as well as file systems for each WSL distribution.
    For the fastest performance speed,[^4] clone Sailbot Workspace in the WSL file system.
    We recommend cloning it somewhere in your Ubuntu user's home directory, which is what Ubuntu opens to by default.

    [^4]: [File storage and performance across file systems](https://learn.microsoft.com/en-us/windows/wsl/filesystems#file-storage-and-performance-across-file-systems){target=_blank}

## 4. Open Sailbot Workspace in VS Code

1. Install `code` command in `PATH`

    === ":material-microsoft-windows: Windows"

        The `code` command is installed by default.

    === ":material-apple: macOS"

        See [launching from the command line](https://code.visualstudio.com/docs/setup/mac#_launching-from-the-command-line){target=_blank}.

    === ":material-linux: Linux"

        The `code` command is installed by default.

2. Run `code sailbot_workspace`

## 5. Open Sailbot Workspace in a Dev Container

1. Ensure that Docker is running
2. Run the `Dev Containers: Reopen in Container` command

## 6. Open the workspace file

1. Open the file `.devcontainer/config/sailbot_workspace.code-workspace` in VS Code
2. Click `Open Workspace`

## 7. Run the `setup` task

!!! info ""

    Moved from Run to Setup page in [:octicons-tag-24: v1.1.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.1.0){target=_blank}

The `setup` task clones the repositories defined in `src/new_project.repos` and updates dependencies of the ROS packages.

??? bug "Can't see the `setup` task"

    If you can't see the `setup` task, run the `Developer: Reload Window` command.
    This may occur when the workspace file is opened for the first time.

## 8. Run the `Build All` task

The `Build All` task builds all the ROS packages.

## 9. Reload the VS Code terminals and window

Delete all open terminals and run the `Developer: Reload Window` command to detect the files that were generated from building.

## Setup Sailbot Workspace in a GitHub Codespace

A codespace is a development environment that's hosted in the cloud.[^5]
Since Sailbot Workspace is resource intensive, it has high hardware requirements and power consumption
which isn't ideal for development on laptops. GitHub Codespaces provides a seamless experience to work on a repository
off-device, especially if the repository has a Dev Container like Sailbot Workspace does. They can run in VS Code
or even in a browser for times when you aren't on your programming computer.

Create a GitHub Codespace following the steps in the relevant GitHub Docs page:
[create a codespace for a repository](https://docs.github.com/en/codespaces/developing-in-codespaces/creating-a-codespace-for-a-repository#creating-a-codespace-for-a-repository){target=_blank}.
A couple things to note:

- For the best Sailbot Workspace development experience, select the high-spec machine available
- There are usage limits if you don't want to pay:
  [monthly included storage and core hours for personal accounts](https://docs.github.com/en/billing/managing-billing-for-github-codespaces/about-billing-for-github-codespaces#monthly-included-storage-and-core-hours-for-personal-accounts){target=_blank}
    - Upgrade to a Pro account for increased usage limits (this is free for students):
      [apply to GitHub Global Campus as a student](https://docs.github.com/en/education/explore-the-benefits-of-teaching-and-learning-with-github-education/github-global-campus-for-students/apply-to-github-global-campus-as-a-student){target=_blank}
    - Stop your codespace as soon as you are done using it:
      [stopping a codespace](https://docs.github.com/en/codespaces/developing-in-codespaces/stopping-and-starting-a-codespace#stopping-a-codespace){target=_blank}
    - Delete codespaces that you do not plan to use anymore:
      [deleting a codespace](https://docs.github.com/en/codespaces/developing-in-codespaces/deleting-a-codespace#deleting-a-codespace){target=_blank}

From there you can follow the setup instructions starting from [6. Open the workspace file](#6-open-the-workspace-file).

Once you have a codespace set up:

- Open it by following the steps in the relevant GitHub Docs page:
[reopening a codespace](https://docs.github.com/en/codespaces/developing-in-codespaces/opening-an-existing-codespace#reopening-a-codespace){target=_blank}
- Close it by running the `Codespaces: Stop Current Codespace` command

!!! warning "Known limitations of running Sailbot Workspace in a GitHub Codespace"

    - Grafana: can't connect to MongoDB
    - Python: can't detect our `custom_interfaces` repository
    - Does not support X11 forwarding to run GUI applications
    - High-spec machines not available: as of March 2023, the highest-spec machine that is publically available
      has a 4-core CPU and 8GB of RAM

[^5]: [GitHub Codespaces overview](https://docs.github.com/en/codespaces/overview){target=_blank}
