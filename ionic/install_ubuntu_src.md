# Source Installation on Ubuntu

These instructions apply to Ubuntu Jammy (22.04) and Ubuntu Noble (24.04).

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and gz-sim in the right order
[colcon](https://colcon.readthedocs.io/en/released/) is recommended.
The colcon tool is available on all platforms using pip (or pip3, if pip fails).

Some tools require Python 3.5 (or higher) which is not the default option on some
platforms (like Ubuntu Focal). The Python
[virtualenv](https://virtualenv.pypa.io/en/latest/) could be a useful solution in
cases where the default option cannot be easily changed.

## Generic tools

Install tools needed by this tutorial:

```bash
sudo apt install python3-pip lsb-release gnupg curl
```

## vcstool and colcon from pip

PIP is available on all platforms:

```bash
pip install vcstool || pip3 install vcstool
```

```bash
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions
```

Check that no errors were printed while installing with PIP. If your system is not recognising the commands, and you're using a system that is compatible with Debian or Ubuntu packages, see the instructions below to install using `apt`.

After installing `vcstool` and `colcon` with PIP, you may need to add their executables to your `$PATH`.
Check where the installation of these packages took place:

```bash
pip show vcstool || pip3 show vcstool | grep Location

pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location
```

If your install path is prefixed with `$HOME/.local`, you'll probably need to add the executables within this directory to your `$PATH` in order to avoid "command not found" errors when using `vcstool` and `colcon` later on:

```bash
export PATH=$PATH:$HOME/.local/bin/
```

## vcstool and colcon from apt

An alternative method is to use the `.deb` packages available on Debian or Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
```

## Git

Gazebo libraries use `git` for version control, so it must be available
in the system for `vcstool` to work properly.

```bash
sudo apt-get install git
```

## Getting the sources

The instructions below use some UNIX commands to manage directories but the
equivalent alternatives on Windows should provide the same result.

The first step is to create a developer workspace in which `vcstool` and
`colcon` can work:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

All the sources of gazebo-ionic are declared in a yaml file. Download
it to the workspace:

```bash
curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-ionic.yaml
```

Use `vcstool` to automatically retrieve all the Gazebo libraries sources from
their repositories:

```bash
vcs import < collection-ionic.yaml
```

The src subdirectory should contain all the sources ready to be built.

## Install dependencies

Before compiling it is necessary to install all the dependencies of the different
packages that compose the Ionic collection. Every platform has a different
method to install software dependencies.

Add `packages.osrfoundation.org` to the apt sources list:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

The command below must be run from a workspace with the Gazebo source code and will install all dependencies in Ubuntu:

```bash
cd ~/workspace/src
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
```

## Building the Gazebo Libraries

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by changing into the workspace and listing the packages
recognized by `colcon`:

```bash
cd ~/workspace/
colcon graph
```

`colcon graph` should list the Gazebo libraries with an
[interdependency diagram](https://colcon.readthedocs.io/en/released/reference/verb/graph.html#example-output).
If that is the case, then you are ready
to build the whole set of libraries:

```bash
colcon build --merge-install
```

To speed up the build process, you could also disable tests by using

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

To use debuggers activate debug symbols. Gazebo will run slower, but you'll be able to use GDB:

```bash
colcon build --cmake-args ' -DBUILD_TESTING=OFF' ' -DCMAKE_BUILD_TYPE=Debug' --merge-install
```

To build a specific package with all its dependent packages:

```bash
colcon build --merge-install --packages-up-to PACKAGE_NAME
```

To build a single package:

```bash
colcon build --packages-select PACKAGE_NAME
```

Visit [colcon documentation](https://colcon.readthedocs.io/en/released/#) to view more `colcon` build and test options.

If there are no errors, all the binaries should be ready to use. You can check the [Troubleshooting](/docs/ionic/troubleshooting#ubuntu) section for errors.

## Using the workspace

The workspace needs to be sourced every time a new terminal is used.

Run the following command to source the workspace in bash:

```bash
. ~/workspace/install/setup.bash
```

Or in zsh:

```zsh
. ~/workspace/install/setup.zsh
```

This is the end of the source install instructions; head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!

## Uninstalling source-based install

A source-based install can be "uninstalled" using several methods, depending on
the results you want:

  1. If you installed your workspace with `colcon` as instructed above, "uninstalling"
     could be just a matter of opening a new terminal and not sourcing the
     workspace's `setup.sh`. This way, your environment will behave as though
     Gazebo is not installed on your system.

  2. If, in addition to not wanting to use the libraries, you're also trying to
     free up space, you can delete the entire workspace directory with:

     ```bash
     rm -rf ~/workspace
     ```

  3. If you want to keep the source code, you can remove the
     `install` / `build` / `log` directories as desired, leaving the `src` directory.

## Troubleshooting

See [Troubleshooting](/docs/ionic/troubleshooting#ubuntu)

## QML Debugging

To perform QML debugging you'll need:

 - Add `--cmake-args -DDQT_QML_DEBUG` flag to colcon
 - QtCreator

You will need to build Gazebo with:

```bash
colcon build --cmake-args -DQT_QML_DEBUG --merge-install
```

> **Note:** Advanced users may note that only the `gz-sim` project needs this flag.

After that's done, launching `gz sim -g` will result in the following message:

```
QML debugging is enabled. Only use this in a safe environment.
QML Debugger: Waiting for connection on port 40000...
```

After that you can just head to
`QtCreator -> Debug -> Start Debugging -> Attach to QML Port...`
and enter the QML port

![](images/GzGuiQmlDebugging01.png)

Once you click there, set the port number to 40000 and hit ok

![](images/GzGuiQmlDebugging02.png)

We're working to improve QtCreator integration so that it works out of the box.

The ruby `gz` script doesn't yet pass the necessary command line arguments to the application.

Note that because all instances will try to use port 40000, only one instance
can use it. If you shutdown the process and restart it immediately too quickly,
the OS may still claim the port is in use and hence the 2nd (re)launch will not
listen to QML debugger attach requests.

### Avoid QML stall waiting for debugger on startup

During development, you may find troublesome that `gz sim -g` won't actually start until
QtCreator hooks to the QML Debugging port.

If that's a problem, you can edit the C++ file `gz-sim/src/gz.cc` and remove `block`
from it. E.g.

```c++
// The following:
const_cast<char *>(
      "-qmljsdebugger=port:40000,block,services:DebugMessages,QmlDebugger,"
      "V8Debugger,QmlInspector,DebugTranslation")

// Must become the following
const_cast<char *>(
      "-qmljsdebugger=port:40000,services:DebugMessages,QmlDebugger,"
      "V8Debugger,QmlInspector,DebugTranslation")
```
