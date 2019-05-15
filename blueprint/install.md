# Blueprint Installation

Blueprint supports the following platforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Gazebo currently only works in headless mode on MacOS.

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The `ign-gazebo`
package is still not available for Windows, the installation should be done from
source code.

## Blueprint Libraries

The Blueprint collection is composed by many different Ignition libraries. The
collection assures that all libraries all compatible and can be used together.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       3.x     |
|   ign-fuel-tools   |       3.x     |
|   ign-gazebo       |       2.x     |
|   ign-gui          |       2.x     |
|   ign-launch       |       1.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       4.x     |
|   ign-physics      |       1.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       2.x     |
|   ign-sensors      |       2.x     |
|   ign-tools        |       0.x     |
|   ign-transport    |       7.x     |
|   sdformat         |       8.x     |

# Option 1: Installation on Ubuntu Bionic

All of the Blueprint binaries are hosted in the osrfoundation repository. To install
all of them, the metapackage `ignition-blueprint` can be installed:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-blueprint
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

# Option 2: Install on MacOS Mojave (10.14)

All the Blueprint binaries are available in Mojave using the [homebrew package manager](https://brew.sh/).
The homebrew tool can easily be installed using:

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

After installing the homebrew package manager, Ignition Blueprint can be installed running:

```bash
brew tap osrf/simulation
brew install ignition-blueprint
```

All libraries should be ready to use and the `ign-gazebo` app ready to be executed.

# Option 3: Source Installation (any platform)

The use of some additional tools is recommended to help with the source compilation, although other ways of correctly getting and building the sources are also possible.

Colcon supports python 3.5 (or higher) which is not the default option in some
platforms (like Ubuntu Bionic). The python [virtualenv](https://virtualenv.pypa.io/en/latest/) could be a useful solution in cases where the default option can not be easily changed.

## Installing vcstool and colcon

For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool). The tool is available from pip
in all platforms:

```bash
pip install vcstool
```

To compile all the different libraries and ign-gazebo in the right order
it is recommended to use [colcon](https://colcon.readthedocs.io/en/released/).
The colcon tool is available in all platforms using pip:

```bash
pip install -U colcon-common-extensions
```

### Use .deb packages in Ubuntu to install vcstool and colcon

An alternative method is to use the .deb packages available on Debian or Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
```

## Getting the sources

The instructions below use some UNIX commands to manage directories but the
obvious alternatives on Windows should provide the same result.

The first step would be to create a developer workspace in which `vcstool` and
`colcon` can work.

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

All the sources of ignition-blueprint are declared in a yaml file. Download
it to the workspace.

```bash
wget https://bitbucket.org/osrf/gazebodistro/raw/default/collection-blueprint.yaml
```

Use `vcstool` to automatically retrieve all the Ignition libraries sources from
their repositories:

```bash
vcs import < collection-blueprint.yaml
```

The src subdirectory should contain all the sources ready to be built.

## Install dependencies

Before compiling it is necessary to install all the dependencies of the diferent
packages that compose the Blueprint collection. Every platform has a different
method to install software dependencies. As reference the command below will
install all dependencies in Ubuntu Bionic (assuming packages.osrfoundation.org
is already in your apt sources list):

```bash
sudo apt-get install cmake freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libdart6-collision-ode-dev libdart6-dev libdart6-utils-urdf-dev libfreeimage-dev libgflags-dev libglew-dev libgts-dev libogre-1.9-dev libogre-2.1-dev libprotobuf-dev libprotobuf-dev libprotoc-dev libqt5core5a libswscale-dev libtinyxml2-dev libtinyxml-dev pkg-config protobuf-compiler python qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings qml-module-qtquick2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev ruby ruby-ronn uuid-dev
```

## Building the Ignition Libraries

The Ignition Libraries require the following compilers on each platform:

* Ubuntu Bionic: gcc 8
* MacOS Mojave: Xcode 10
* Windows: Visual Studio 2017

### Installing gcc version 8 on Ubuntu Bionic

To install `gcc` version 8 on Ubuntu Bionic:

```bash
sudo apt-get install g++-8
```

Set `gcc-8` and `g++-8` to be the default compilers.

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
```

At this point `gcc`  and `g++` should both report version 8. Test this with
the following commands.

```bash
gcc -v
g++ -v
```

### Building the colcon workspace

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by changing into the workspace and listing the packages
recognized by `colcon`:

```bash
cd ~/workspace/
colcon list -g
```

`colcon` should list the Ignition libraries with their
interdependencies. If that is the case, then you are ready
to build the whole set of libraries:

```bash
colcon build --merge-install
```

If there are no errors, all the binaries should be ready to use.

## Using the workspace

The workspace binaries are ready but every time that `ign gazebo` needs to be
executed or third party code is going to be developed using the Ignition
libraries, one command is needed:

```bash
. ~/workspace/install/local_setup.bash
(or call ~/workspace/install/local_setup.bat on Windows)
```

After running the command all paths for running apps or developing code
will be set in the current shell.
