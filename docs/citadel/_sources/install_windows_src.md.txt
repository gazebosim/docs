<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Source Installation on Windows 10

Currently, `ign-gazebo` and `ign-launch` are not supported on Windows 10.

Additionally, command line tools, DART physics engine, and GUI capabilities are
not currently supported in Windows. These functionalities correspond to the currently
building packages `ign-tools`, `ign-physics`, and `ign-gui`, respectively.

**Note**

You will still be able to use `TPE` as a physics engine
(see [here](https://gazebosim.org/api/physics/2.2/physicsplugin.html) for more information on `TPE`).

## Install dependencies

1. Install a [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
   Miniconda suffices. You will likely want to check the box to add `conda` to your `PATH`
   during the installation process so that you won't have to do this step manually.

2. Install [Visual Studio 2019](https://visualstudio.microsoft.com/downloads/).
   The Community version is free for students, open-source developers, and personal
   development. Check "Desktop development with C++" in the Workloads tab,
   and uncheck "C++ Cmake Tools." We will install cmake via Conda.

3. Open a Visual Studio Command Prompt (search for "x64 Native Tools Command Prompt
   for VS 2019" in the Windows search field near the Windows button). Optionally,
   right-click and pin to the task bar for quick access in the future.

  If you did not add Conda to your `PATH` environment variable during Conda installation,
  you may need to navigate to the location of `condabin` in order to use the `conda` command.
  To find `condabin`, search for "Anaconda Prompt" in the Windows search field near the
  Windows button, open it, run `where conda`, and look for a line containing the directory `condabin`.

4. Navigate to your `condabin`, if necessary, and then create and activate a Conda environment:
  ```bash
  conda create -n ign-ws
  conda activate ign-ws
  ```

  Once you have activate an environment, a prefix like `(ign-ws)` will be prepended to
  your prompt, and you can use the `conda` command outside of the `condabin` directory.

  You can use `conda info --envs` to see all of your environments.

  To remove an environment, use `conda env remove --name <env_name>`.

5. Install dependencies:

  ```bash
  conda install cmake git vcstool curl pkg-config ^
  colcon-common-extensions eigen freeimage gts ^
  glib dlfcn-win32 ffmpeg ruby tinyxml2 tinyxml ^
  protobuf urdfdom zeromq cppzmq ogre jsoncpp ^
  libzip qt --channel conda-forge
  ```

6. Navigate to where you would like to build the library, create and enter your workspace directory,
   create the `src` directory which will contain the Ignition source code, and then clone the repositories.
  ```bash
  mkdir ign-ws
  cd ign-ws
  curl -O https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-citadel.yaml
  mkdir src
  vcs import src < collection-citadel.yaml
  ```

## Building the Ignition Libraries

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by navigating to your workspace and listing the packages
recognized by `colcon`:

```bash
colcon graph
```

`colcon graph` should list the Ignition libraries with an
[interdependency diagram](https://colcon.readthedocs.io/en/released/reference/verb/graph.html#example-output).
If that is the case, then you are ready to build the whole set of libraries:

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --packages-up-to ignition-gazebo3
```
Tests are turned off as they are not currently supported on Windows.

**Note:** All of the Ignition packages up to, but not including `ign-gazebo`
are currently building.  The above command should successfully build all packages except for `ign-gazebo`.

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

The workspace needs to be sourced every time a new terminal is used.

Run the following command to source the workspace:

```bash
call install\setup.bat
```

This is the end of the source install instructions; head back to the [Getting started](/docs/all/getstarted)
page to start using Ignition!

## Uninstalling source-based install

A source-based install can be "uninstalled" using several methods, depending on
the results you want:

  1. If you installed your workspace with `colcon` as instructed above, "uninstalling"
     could be just a matter of opening a new terminal and not sourcing the
     workspace's `setup.bat`. This way, your environment will behave as though
     there is no Ignition install on your system.

  2. If, in addition to not wanting to use the libraries, you're also trying to
     free up space, you can delete the entire workspace directory from within
     your `conda` environment with:

     ```bash
     rm -rf <workspace_name>
     ```

  3. If you want to keep the source code, you can remove the
     `install` / `build` / `log` directories as desired, leaving the `src` directory.

## Troubleshooting

See [Troubleshooting](/docs/citadel/troubleshooting)
