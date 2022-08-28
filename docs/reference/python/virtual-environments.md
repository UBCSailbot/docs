# Virtual Environments

The Python virtual environment is a tool for dependency management and project isolation. They solve many
common issues, including:

- **Dependency Resolution:** A project might want a package with version A while another project might want
a package with version B. With a virtual environment, you can separate which packages that you want to use
for a given project.

- **Project Isolation:** The environment for your project is self-contained and reproducible by capturing all
dependencies in a configuration file.

- **Housekeeping:** Virtual environments allow you to keep your global workspace tidy.

There are two main methods of creating virtual environments: [virtualenv](https://pypi.org/project/virtualenv/)
and [Anaconda](https://www.anaconda.com/). Each have their own benefits and drawbacks. Here are some differences
between the two:

| Virtualenv                                        | Anaconda                                                 |
| :------------------------------------------------ | :------------------------------------------------------- |
| Environment files are modular.                    | Environment files are available globally.                |
| Must activate environment at the location of the files. | Can activate the environment anywhere.             |
| Can only use `pip` to install packages.           | Can either use `pip` or built-in `conda` package manager.|
| Installation is very simple.                      | Installation takes more effort.                          |
| Can only install python packages.                 | In addition to packages, you can also control the python version and IDE settings.|
| Using `pip` to install packages usually works and is easy to use.    | Using `conda` package manager is more tedious since commands may vary. |

For the purposes of UBC Sailbot, we recommend **virtualenv** over Anaconda. However, feel free to appeal to your preferences.

## Installation

=== "Virtualenv"

    If you already have python and the pip package manager installed, just execute the following:

    ```bash title="Using pip to install virtualenv"
    pip install virtualenv
    ```

=== "Anaconda"

    Go to the official [Anaconda website](https://www.anaconda.com/) and follow the installation instructions 
    for your operating system.

## Using virtual environments

### Creating a virtual environment

=== "Virtualenv"

    Since virtualenv creates the environment directory in a specific location, make sure that you
    are in the location that you want to create it in (you could always move the folder elsewhere later on).

    ```bash title="Create virtual environment with virtualenv"
    # Go to desired location
    cd <PATH TO DIRECTORY>

    # Create the environment
    python3 -m venv <ENV NAME>
    ```

    Verify that your environment is created by examining your current directory and look for the directory
    that matches the name of your virtual environment.

=== "Anaconda"

    Since the environment will be available globally, there is no need to go to a specific location to create
    it.

    ``` bash title="Create virtual environment with Anaconda"
    # Create environment with name and python version
    conda env create -n <ENV NAME> python=<PYTHON VERSION NUM>
    ```

    If you don't specify a python version, the default is the latest version. Verify that your environment is created
    by executing `conda env list`.

### Activating the virtual environment

To use the virtual environment, you must activate it.

=== "Virtualenv"

    ```batch title="Activation for Windows"
    <ENV NAME>\Scripts\activate
    ```

    ```bash title="Activation for MacOS or Linux"
    source <ENV NAME>/bin/activate
    ```

=== "Anaconda"

    ```bash title="Activation for Anaconda"
    conda activate <ENV NAME>
    ```

After activating your virtual environment, you might see `(<ENV NAME>)` on your terminal before or after
your current line. Now you are in your virtual environment!

### Installing dependencies

Any dependencies that you install while your virtual environment is activated are only available in your virtual
environment. If you deactivate your environment and try to use those dependencies, you will find that you will get
errors because they will not be found unless you install those dependencies in the other environment!

=== "Virtualenv"

    Use the `pip` package manager to install python dependencies.

    ```bash title="Install dependency with pip"
    pip install <PACKAGE>
    ```

=== "Anaconda"

    Use the built-in `conda` package manager to install python dependencies.

    ```bash title="Install dependency with conda"
    conda install -c <CHANNEL> <PACKAGE>
    ```

    Sometimes, installing a package like this simply won't work because you are not installing
    from the correct [channel](https://conda.io/projects/conda/en/latest/user-guide/concepts/channels.html).
    You usually will have to google the command to use in order to install your package correctly because
    it usually comes from a specific channel that you don't know about. Some common channels to try are:

    - conda-forge
    - anaconda
    - bioconda
    - r

    Not specifying a channel directs you to the "default" channel.

    A foolproof method (usually) is to use the `pip` package manager instead. Some packages are exclusively
    available via the `pip` package manager, so you may be forced to use it in some instances anyways.

    ```bash title="Install dependency with pip in Anaconda"
    # First install pip
    conda install pip

    # Install dependency with pip
    pip install <PACKAGE>
    ```

### Deactivating the virtual environment

When you are finished using your virtual environment, you will need to deactivate it.

=== "Virtualenv"

    ```bash title="Deactivate virtualenv environment"
    deactivate
    ```

=== "Anaconda"

    ```bash title="Deactivate anaconda environment"
    conda deactivate
    ```

## Reproducing your virtual environment

When you want to share your code with others, it is important for others to be able to reproduce the environment
that you worked in. We discuss two topics in this section: exporting your environment and reproducing the environment.

### Exporting your virtual environment

In order to reproduce your virtual environment, you need to export some information about your environment.
Be sure to follow the instructions below **while your environment is activated**.

=== "Virtualenv"

    You will create a `requirements.txt` file, which essentially lists all of your python dependencies in one
    file:

    ```bash title="Creating requirements file"
    pip freeze > requirements.txt
    ```

    The `pip freeze` command prints all of your pip dependencies, and `> requirements.txt` redirects the output
    to a text file.

=== "Anaconda"

    Anaconda uses configuration files to recreate an environment.

    **Windows**

    Execute the following command to create a file called `environment.yml`:
    
    ```batch title="Create config file"
    conda env export > environment.yml
    ```

    Then, open the `environment.yml` file and delete the line with `prefix:`.

    **MacOS or Linux**

    Execute the following command to create a file called `environment.yml`:

    ```bash title="Create config file"
    conda env export | grep -v "^prefix: " > environment.yml
    ```

### Reproducing the environment

You can reproduce your virtual environment when given the information about it. The steps above tell you how
to extract the information, and now we will use that information to recreate the virtual environment.
Remember to **deactivate the current environment** before making a new environment.

=== "Virtualenv"

    We use the `requirements.txt` file that we generated earlier to recreate the environment.

    ```bash title="Recreate virtualenv environment"
    # Create the new environment
    python -m venv <NEW ENV NAME>
    
    # Activate the environment
    source <NEW ENV NAME>/bin/activate

    # Install dependencies
    pip install -r <PATH TO requirements.txt file>
    ```

=== "Anaconda"

    We use the `environment.yml` file that we generated earlier to recreate the environment.

    ```bash title="Recreate the conda environment"
    # Create the new environment with the dependencies
    conda env create -f <PATH TO environment.yml>
    ```

    !!! info

        If you create the new environment without specifying a name, you might get an error saying that
        the environment already exists because by default, the `environment.yml` will give the name
        of the environment that it was created from. To avoid this, add `-n <ENV NAME>` to the above
        command to rename the environment.

## Official references

In this section, we summarized what virtual environments are, why they are used, and how to use them. We did not
cover all of the functions of virtual environments, but feel free to consult the official references to learn
about virtual environments more in depth.

- [Virtualenv Reference](https://docs.python.org/3/library/venv.html#module-venv)
- [Anaconda Reference](https://conda.io/projects/conda/en/latest/user-guide/concepts/environments.html#)
