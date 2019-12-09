# Citadel Installation

Citadel supports the following platforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Ignition currently only works in headless mode

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The `ign-gazebo`
package is still not available for Windows, the installation should be done from
source code.

## Citadel Libraries

The Citadel collection is composed by many different Ignition libraries. The
collection assures that all libraries all compatible and can be used together.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       3.x     |
|   ign-fuel-tools   |       4.x     |
|   ign-gazebo       |       3.x     |
|   ign-gui          |       3.x     |
|   ign-launch       |       2.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       5.x     |
|   ign-physics      |       2.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       3.x     |
|   ign-sensors      |       3.x     |
|   ign-tools        |       1.x     |
|   ign-transport    |       8.x     |
|   sdformat         |       9.x     |

# Option 1: Binary Installation on Ubuntu Bionic

All of the Citadel binaries are hosted in the osrfoundation repository. To install
all of them, the metapackage `ignition-citadel` can be installed:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-citadel
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

# Option 2: Binary Installation on MacOS Mojave (10.14)

All the Citadel binaries are available in Mojave using the [homebrew package manager](https://brew.sh/).
The homebrew tool can easily be installed using:

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

After installing the homebrew package manager, Ignition Citadel can be installed running:

```bash
brew tap osrf/simulation
brew install ignition-citadel
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

# Option 3: Source Installation 

The use of some additional tools is recommended to help with the source compilation, although other ways of correctly getting and building the sources are also possible.

Colcon supports python 3.5 (or higher) which is not the default option in some
platforms (like Ubuntu Bionic). The python [virtualenv](https://virtualenv.pypa.io/en/latest/) could be a useful solution in cases where the default option can not be easily changed.

## Linux

### Installing vcstool and colcon

For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool). The tool is available from pip
in all platforms:

```bash
pip install vcstool
```

Since Ignition libraries use `mercurial` for version control it must be available in the system for `vcstool` to work properly. While `mercurial` is available via pip, Python 3 support is currently in beta. Therefore a different means of installation is recommended. In Ubuntu:

```bash
sudo apt-get install mercurial
```

To compile all the different libraries and ign-gazebo in the right order
it is recommended to use [colcon](https://colcon.readthedocs.io/en/released/).
The colcon tool is available in all platforms using pip (or pip3, if pip fails):

```bash
pip install -U colcon-common-extensions
```

### Use .deb packages in Ubuntu to install vcstool and colcon

An alternative method is to use the .deb packages available on Debian or Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions mercurial
```

### Install dependencies

Before compiling it is necessary to install all the dependencies of the diferent
packages that compose the Citadel collection. Every platform has a different
method to install software dependencies. 

Add packages.osrfoundation.org to the apt sources list:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

As reference the command below will
install all dependencies in Ubuntu Bionic:

```bash
sudo apt-get install cmake freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libdart6-collision-ode-dev libdart6-dev libdart6-utils-urdf-dev libfreeimage-dev libgflags-dev libglew-dev libgts-dev libogre-1.9-dev libogre-2.1-dev libprotobuf-dev libprotobuf-dev libprotoc-dev libqt5core5a libswscale-dev libtinyxml2-dev libtinyxml-dev pkg-config protobuf-compiler python qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings qml-module-qtquick2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtqml-models2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev ruby ruby-ronn uuid-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libzmq3-dev libsqlite3-dev libwebsockets-dev swig ruby-dev -y
```

## Mac OS Mojave (10.14)

Tools and dependencies for Citadel will be installed using the [homebrew package manager](https://brew.sh/).
The homebrew tool can easily be installed by entering the following in a terminal:

```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

### Install python3, vcstool, mercurial and colcon

Citadel is compatible with Python3; it can be installed by running the following in a terminal:

```bash
brew install python3
```


For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool). The tool is available from pip
in all platforms:

```bash
python3 -m pip install vcstool
```

Since Ignition libraries use `mercurial` for version control it must be available in the system for `vcstool` to work properly. While `mercurial` is available via pip, Python 3 support is currently in beta. Therefore a different means of installation is recommended. In Ubuntu:

```bash
brew install mercurial
```

To compile all the different libraries and ign-gazebo in the right order
it is recommended to use [colcon](https://colcon.readthedocs.io/en/released/).
The colcon tool is available in all platforms using pip (or pip3, if pip fails):

```bash
python3 -m pip install -U colcon-common-extensions
```

### Install dependencies

Add `osrf/simulation` to Homebrew formulae

```bash
brew update
brew tap osrf/simulation
```

Install all dependencies

Dependency for Ogre:

```bash
brew cask install xquartz
```

General dependencies:

```bash
brew install assimp boost bullet cmake cppzmq dartsim@6.10.0 doxygen eigen fcl ffmpeg flann freeimage freetype gflags google-benchmark gts ipopt irrlicht jsoncpp libccd libyaml libzzip libzip nlopt ode open-scene-graph ossp-uuid ogre1.9 ogre2.1 pkg-config protobuf qt qwt rapidjson ruby tbb tinyxml tinyxml2 urdfdom zeromq
```

`dartsim@6.10.0` and `qt5` are not sym-linked, to use those dependencies when building `ignition-physics2` and `ignition-gui3`, run the following after installation to add them to `/use/local`:

```bash
# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
# qt5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt
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

All the sources of ignition-citadel are declared in a yaml file. Download
it to the workspace.

```bash
wget https://bitbucket.org/osrf/gazebodistro/raw/default/collection-citadel.yaml
```

Use `vcstool` to automatically retrieve all the Ignition libraries sources from
their repositories:

```bash
vcs import < collection-citadel.yaml
```

The src subdirectory should contain all the sources ready to be built.


## Building the Ignition Libraries

The Ignition Libraries require the following compilers on each platform:

* Ubuntu Bionic: gcc 8
* MacOS Mojave: Xcode 10
* Windows: Visual Studio 2017

### Linux: Installing gcc version 8 on Ubuntu Bionic

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

### Mac OS: Installing gcc

On Mac machines, gcc is acquired by installing Xcode command line tools. The required version of Xcode for Citadel is Xcode 10.3, which can be downloaded from [Apple Developer Site](https://developer.apple.com/download/more/). You will need to sign in to your Apple account and download the Mojave version of Xcode command line tools. Command line tools can also be obtained by downloading Xcode from the Apple App Store, but installing the full app may take over an hour.


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

To speed up the build process, you could also disable tests by using 

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

To build a package with all its dependent packages:

```bash
colcon build --merge-install --packages-up-to PACKAGE_NAME
```

To build a single package:

```bash
colcon build --packages-select PACKAGE_NAME
```

Visit [colcon documentation](https://colcon.readthedocs.io/en/released/#) to view more `colcon` build and test options.

If there are no errors, all the binaries should be ready to use.

## Using the workspace (binary)

The workspace binaries are ready but every time that `ign gazebo` needs to be
executed or third party code is going to be developed using the Ignition
libraries, one command is needed:

```bash
. ~/workspace/install/local_setup.bash
(or call ~/workspace/install/local_setup.bat on Windows)
```

After running the command all paths for running apps or developing code
will be set in the current shell.

## Using the workspace (from source)

The workspace needs to be sourced everytime a new terminal is used.

Run the following command to souce the workspace in bash

```bash
. ~/workspace/install/setup.bash
```

or in zsh

```zsh
. ~/workspace/install/setup.zsh
```

## TroubleShooting

### OSX: Unable to find `urdf_model.h` error

After installing all the dependencies and starting the build process, you may encounter an error looks like this:

```bash
/Users/user/citadel_ws/src/sdformat/src/parser_urdf.cc:30:10: fatal error: 'urdf_model/model.h' file not found
#include <urdf_model/model.h>
         ^~~~~~~~~~~~~~~~~~~~
1 error generated.
make[2]: *** [src/CMakeFiles/sdformat9.dir/parser_urdf.cc.o] Error 1
make[1]: *** [src/CMakeFiles/sdformat9.dir/all] Error 2
make: *** [all] Error 2
Failed   <<< sdformat9	[ Exited with code 2 ]
```

First check if `urdfdom` and `urdfdom_headers` are installed by running

```bash
brew install urdfdom urdfdom_headers
```

Then if the error persists, compile with the internal version of `urdfdom` by running

```bash
colcon build --cmake-args -DUSE_INTERNAL_URDF=ON --merge-install
```

This command will ignore the system installation of `urdfdom` and use the internal version instead.
