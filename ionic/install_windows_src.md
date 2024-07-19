<div class="warning">
WARNING: Current Windows support is experimental.
</div>

# Source Installation on Windows 10 or 11

OGRE2 rendering capabilities are not currently supported in Windows, and Gazebo GUI
works in a limited fashion. These functionalities correspond to the currently
building packages `gz-rendering` and `gz-sim`, respectively. The packages will build,
but you can expect runtime failures when using their functionalities.

> **NOTE**
> You should be able to use `ogre` as a rendering engine instead of the default `ogre2`.
> Just append `--render-engine ogre` to the command line.

## Install dependencies

1. Install a conda distribution. As Gazebo uses all dependencies from the conda-forge channel,
   we suggest to install miniforge following [the official miniforge installation docs](https://github.com/conda-forge/miniforge#windows)
   You will likely want to check the box to add `conda` to your `PATH`
   during the installation process so that you won't have to do this step manually.

2. Install [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/).
   The Community version is free for students, open-source developers, and personal
   development. Check "Desktop development with C++" in the Workloads tab,
   check "MFC and ATL support", and uncheck "C++ Cmake Tools." We will install
   cmake via Conda. All other checkboxes can be left unchecked.

3. Open a Visual Studio Command Prompt (search for "x64 Native Tools Command Prompt
   for VS" in the Windows search field near the Windows button). Optionally,
   right-click and pin to the task bar for quick access in the future.

   If you did not add Conda to your `PATH` environment variable during Conda installation,
   you may need to navigate to the location of `condabin` in order to use the `conda` command.
   To find `condabin`, search for "Anaconda Prompt" in the Windows search field near the
   Windows button, open it, run `where conda`, and look for a line containing the directory `condabin`.

4. Navigate to your `condabin`, if necessary, and then create and activate a Conda environment:
   ```bash
   conda create -n gz-ws
   conda activate gz-ws
   ```
   Once you have activated an environment, a prefix like `(gz-ws)` will be prepended to
   your prompt, and you can use the `conda` command outside of the `condabin` directory.

   You can use `conda info --envs` to see all of your environments.

   To speed up conda installations, also set the following to use libmamba solver.
   Older conda installations may need to do [additional steps](https://www.anaconda.com/blog/a-faster-conda-for-a-growing-community).
   ```bash
   conda config --set solver libmamba
   ```
   To remove an environment, use `conda remove --all --name <env_name>`.

   > **NOTE**
   > This way of Conda environment creation puts it into a default folder. If you need
     to install it elsewhere, use `--prefix <env_path>` instead of `--name <env_name>`.
     Environments in custom paths cannot be referenced by names, so even `conda activate`
     needs to be passed a path (relative or absolute) instead of the name. If you refer
     to a subdirectory of the current directory, you have to prepend `.\` so that Conda
     knows it is a path and not a name.

5. Install dependencies:
   ```bash
   conda install cmake git vcstool curl pkg-config ^
   colcon-common-extensions dartsim eigen freeimage gdal gts ^
   glib dlfcn-win32 ffmpeg ruby tinyxml2 tinyxml ^
   libprotobuf urdfdom zeromq cppzmq ogre=1.10 ogre-next jsoncpp ^
   libzip qt pybind11 --channel conda-forge
   ```
   This can take tens of minutes (or less when using libmamba solver).

6. Navigate to where you would like to build the library, create and enter your workspace directory,
   create the `src` directory which will contain the Gazebo source code.
   ```bash
   mkdir gz-ws
   cd gz-ws
   mkdir src
   ```

7. Then clone the repositories
   ```bash
   # CMD
   curl -sk https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-ionic.yaml -o collection-ionic
   vcs import src < collection-ionic

   # PowerShell
   curl https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-ionic.yaml -o collection-ionic
   vcs import --input collection-ionic src
   ```

## Building the Gazebo Libraries

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
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --packages-up-to gz-sim9 gz-tools2
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
and Conda environment activated before that).

Run the following command to source the workspace:

```bash
# CMD
call install\setup.bat

# PowerShell
.\install\setup.ps1
```

This is the end of the source install instructions; head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!

> **NOTE**
> As Gazebo GUI is not yet working, running `gz sim` will not work. You can run only the server with
> ```batch
> gz sim -s -v
> ```

> **NOTE**
> If your username contains spaces (which is quite common on Windows), you will probably get errors
>  saying `Invalid partition name [Computer:My User With Spaces]`. Fix this by changing `GZ_PARTITION`
>  to something else:
> ```batch
> set GZ_PARTITION=test
> ```
> Remember to set the same partition in all other consoles.

### Gazebo GUI workaround

Although running `gz sim` without arguments is not supported on Windows,
and `gz sim -g` is also not supported, there is a workaround you can apply
to be able to launch `gz sim -g` on Windows.

> Manually comment [these lines](https://github.com/gazebosim/gz-sim/blob/gz-sim7_7.5.0/src/cmd/cmdsim.rb.in#L497-L501) and [these lines](https://github.com/gazebosim/gz-sim/blob/gz-sim7_7.5.0/src/cmd/cmdsim.rb.in#L558-L562) in file `install\lib\ruby\gz\cmdsim9.rb`.

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
     rm -rf <workspace_name>
     ```

  3. If you want to keep the source code, you can remove the
     `install` / `build` / `log` directories as desired, leaving the `src` directory.

  4. Last, if you do not need the conda environment anymore, you can remove it with

     ```bash
     conda deactivate
     conda remove --all --name <env_name>
     # or conda remove --all --prefix <path_to_env> if you installed to custom path
     ```

## Troubleshooting

See [Troubleshooting](/docs/ionic/troubleshooting#windows)
