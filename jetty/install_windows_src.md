<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Source Installation on Windows 10 or 11

OGRE2 rendering capabilities are supported in Windows, and Gazebo GUI
works in a limited fashion. These functionalities correspond to the currently
building packages `gz-rendering` and `gz-sim`, respectively. The packages will build,
without any failures when using their functionalities.

> **NOTE**
> You should be able to use `ogre` as a rendering engine instead of the default `ogre2`.
> Just append `--render-engine ogre` to the command line.

## Dependency distribution (conda-forge) and the package manager (Pixi)

Binaries for all the dependendencies used by Gazebo can be found in the [conda-forge](https://conda-forge.org/)
package repository. The Gazebo buildfarm and these instructions uses the [Pixi](https://pixi.sh/) package manager
but other package managers for Conda should work like miniforge, mamba, etc.

## Install dependencies

1. If no package manager is installed in the system to manage conda-forge dependencies, the recommended
   option is to follow the Pixi installer instructions from https://pixi.sh/latest/ to install pixi.
   Once pixi has been installed, close the terminal session and start it again,
   which will ensure pixi is on the PATH.

3. Install [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/).
   The Community version is free for students, open-source developers, and personal
   development. Check "Desktop development with C++" in the Workloads tab,
   check "MFC and ATL support", and uncheck "C++ Cmake Tools." We will install
   cmake via Conda. All other checkboxes can be left unchecked.

4. Open a Visual Studio Command Prompt (search for "x64 Native Tools Command Prompt
   for VS" in the Windows search field near the Windows button) or Developer PowerShell
   for VS (search for "developer powershell"). Optionally,
   right-click and pin to the task bar for quick access in the future.

5. Create the Pixi project directory and download the configuration files
   Pixi projects operates inside a given directory, creating one in any user system location
   should be enough. There are two configuration files to download:
   ```bash
   mkdir gazebo
   cd gazebo
   curl.exe -L -O https://raw.githubusercontent.com/gazebo-tooling/release-tools/refs/heads/master/conda/envs/noble_like/pixi.toml
   curl.exe -L -O https://raw.githubusercontent.com/gazebo-tooling/release-tools/refs/heads/master/conda/envs/noble_like/pixi.lock
   ```
6. Install dependencies using Pixi and enable the Pixi environment:
   Once inside the Pixi project directory (i.e: `gazebo`), Pixi can install the dependencies
   that are part of the configuration file:
   ```bash
   :: inside the gazebo directory
   pixi install
   ```
   This can take some minutes. After this Pixi can make accesible all the libraries and dependencies
   installed by providing a shell enviroment:
   ```bash
   :: inside the gazebo directory
   pixi shell
   ```
   With the Pixi shell environment active (there will be a "(gazebo)" label before the terminal prompt) the Pixi environment
   is accesible from anywhere in the file system.

7. Navigate to where you would like to build the library, create and enter your workspace directory,
   create the `src` directory which will contain the Gazebo source code.
   ```bash
   mkdir gz-ws
   cd gz-ws
   mkdir src
   ```

8. Then clone the repositories
   ```bash
   vcs import --input https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-jetty.yaml src/
   ```

## Building the Gazebo Libraries

> **NOTE**
> Be sure of being under a Visual Studio Developer shell (step 3 above) and
> after executing the pixi shell (step 5 above).

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by navigating to your workspace and listing the packages
recognized by `colcon`:

```bash
colcon graph
```

`colcon graph` should list the Gazebo libraries with an
[interdependency diagram](https://colcon.readthedocs.io/en/released/reference/verb/graph.html#example-output).
If that is the case, then you are ready to build the whole set of libraries:

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF -DSKIP_SWIG=ON --merge-install --packages-up-to gz-sim gz-tools2
```
Tests are turned off as they are not currently supported on Windows.

To build a specific package with all its dependent packages:

```bash
colcon build --merge-install --packages-up-to PACKAGE_NAME
```

To build a single package:

```bash
colcon build --packages-select PACKAGE_NAME
```

Visit [colcon documentation](https://colcon.readthedocs.io/en/released/#) to view more `colcon` build and test options.

If there are no errors, all the binaries should be ready to use.

## Using the workspace

The workspace needs to be sourced every time a new terminal is used (
and the Pixi environment activated before that).

The overall instructions for setting up a new terminal to use the built
workspace are:

```bash
# CMD
cd gazebo
pixi shell
call install\setup.bat

# PowerShell
cd gazebo
pixi shell
.\install\setup.ps1
```
<div class="warning">
An issue in the conda-forge Qt6 package is requiring to set QT environment variables:
</div>

```bash
# Until https://github.com/conda-forge/qt-main-feedstock/issues/275 is resolved
cd gazebo
set QT_PLUGIN_PATH=%CONDA_PREFIX%\Library\lib\qt6\plugins\platforms
set QML2_IMPORT_PATH=%CONDA_PREFIX%\Library\lib\qt6\qml
```

You should now be able to launch gazebo normally:

```bash
gz sim --verbose
```

Alternativally launching the server and the client in two different terminales (after sourcing
in both the install scripts, the pixi shell and the QT env variables):

```bash
# Launch server in one terminal
gz sim -s

# In separate terminal, launch gui
gz sim -g
```

This is the end of the source install instructions; head back to the [Getting started](getstarted)
page to start using Gazebo!

> **NOTE**
> If your username contains spaces (which is quite common on Windows), you will probably get errors
>  saying `Invalid partition name [Computer:My User With Spaces]`. Fix this by changing `GZ_PARTITION`
>  to something else:
> ```bat
> set GZ_PARTITION=test
> ```
> Remember to set the same partition in all other consoles.

### Gazebo GUI workaround

Although running `gz sim` without arguments is not supported on Windows,
 the `gz sim -g` command is  supported, and you can use it to launch the graphical interface on Windows.


This should allow you to run the GUI in a separate console, connecting to the server running in another console.

## Uninstalling source-based install

A source-based install can be "uninstalled" using several methods, depending on
the results you want:

  1. If you installed your workspace with `colcon` as instructed above, "uninstalling"
     could be just a matter of opening a new terminal and not sourcing the
     workspace's `setup.bat`. This way, your environment will behave as though
     Gazebo is not installed on your system.

  2. If, in addition to not wanting to use the libraries, you're also trying to
     free up space, you can delete the entire workspace directory from within
     your `conda` environment with:

     ```bash
     rmdir /s /q <workspace_name>  # gz-ws was used as workspace_name in this document
     ```

  3. If you want to keep the source code, you can remove the
     `install` / `build` / `log` directories as desired, leaving the `src` directory.

  4. Last, if you do not need the Pixi environment anymore, you can remove it with

     ```bash
     rmdir /s /q <pixi_env_path>  # "gazebo" was used as pixi_env_path in this document
     ```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#windows)
