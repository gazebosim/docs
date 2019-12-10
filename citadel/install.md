# Citadel Installation

Citadel supports the following platforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Ignition currently only works in headless mode
     * Headless mode: GUI does not render; instead of using `ign gazebo fuel.sdf` command, use `ign gazebo -s fuel.sdf` to start the server only.

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The `ign-gazebo`
package is still not available for Windows, the installation should be done from
source code.

## Citadel Libraries

The Citadel collection is composed by many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

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

# Option 1: Binary Installation

Binary installation is the recommended method of installing Ignition.

##  Ubuntu Bionic

All of the Citadel binaries are hosted in the osrfoundation repository. To install
all of them, the metapackage `ignition-citadel` can be installed:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-citadel
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

## MacOS Mojave (10.14)

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

This is the end of the binary install instructions; head back to the [Getting started](getting_started.html)
page to start using Ignition!

# Option 2: Source Installation

Source installation is recommended for users planning on altering Ignition's source code (advanced).

## Ubuntu Bionic

### Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and ign-gazebo in the right order
it is recommended to use [colcon](https://colcon.readthedocs.io/en/released/).
The colcon tool is available in all platforms using pip (or pip3, if pip fails).

Some tools require Python 3.5 (or higher) which is not the default option in some
platforms (like Ubuntu Bionic). The Python
[virtualenv](https://virtualenv.pypa.io/en/latest/) could be a useful solution in
cases where the default option can not be easily changed.

### vcstool and colcon from pip

PIP is available on all platforms:

```bash
pip install vcstool
```

```bash
pip install -U colcon-common-extensions
```

### vcstool and colcon from apt

An alternative method is to use the `.deb` packages available on Debian or Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions mercurial
```

### Mercurial

Ignition libraries use `mercurial` for version control, so it must be available
in the system for `vcstool` to work properly. While `mercurial` is available via pip,
Python 3 support is currently in beta. Therefore a different means of installation
is recommended. In Ubuntu:

```bash
sudo apt-get install mercurial
```

### Getting the sources

The instructions below use some UNIX commands to manage directories but the
equivalent alternatives on Windows should provide the same result.

The first step is to create a developer workspace in which `vcstool` and
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


### Install dependencies

Before compiling it is necessary to install all the dependencies of the different
packages that compose the Citadel collection. Every platform has a different
method to install software dependencies.

Add packages.osrfoundation.org to the apt sources list:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

The command below will install all dependencies in Ubuntu Bionic:

```bash
sudo apt-get install cmake freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libdart6-collision-ode-dev libdart6-dev libdart6-utils-urdf-dev libfreeimage-dev libgflags-dev libglew-dev libgts-dev libogre-1.9-dev libogre-2.1-dev libprotobuf-dev libprotobuf-dev libprotoc-dev libqt5core5a libswscale-dev libtinyxml2-dev libtinyxml-dev pkg-config protobuf-compiler python qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings qml-module-qtquick2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtqml-models2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev ruby ruby-ronn uuid-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libzmq3-dev libsqlite3-dev libwebsockets-dev swig ruby-dev -y
```

#### Install compiler requirements

The Ignition Libraries require the gcc 8 compiler on Ubuntu Bionic.
(Windows requires Visual Studio 2019).

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

This is the end of the Ubuntu-specific source installation instructions. Continue
to the section "Building the Ignition Libraries" to complete the installation.

## MacOS Mojave (10.14)

### Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and ign-gazebo in the right order
it is recommended to use [colcon](https://colcon.readthedocs.io/en/released/).
The colcon tool is available in all platforms using pip (or pip3, if pip fails).


### Python3 and mercurial from homebrew

Tools and dependencies for Citadel can be installed using the [homebrew package manager](https://brew.sh/).
The homebrew tool can be installed by entering the following in a terminal:

```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

Citadel is compatible with Python3; it can be installed by running the following in a terminal:

```bash
brew install python3
```

Ignition libraries use `mercurial` for version control, so it must be available
in the system for `vcstool` to work properly. While `mercurial` is available via pip,
Python 3 support is currently in beta. Therefore a different means of installation
is recommended. In MacOS:

```bash
brew install mercurial
```

### vcstool and colcon from pip

PIP is available on all platforms:

```bash
python3 -m pip install vcstool
```

```bash
python3 -m pip install -U colcon-common-extensions
```

### Getting the sources

The first step is to create a developer workspace in which `vcstool` and
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

`dartsim@6.10.0` and `qt5` are not sym-linked, to use those dependencies when building
`ignition-physics2` and `ignition-gui3`, run the following after installation to add them to `/use/local`:

```bash
# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
# qt5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt
```

#### Install compiler requirements

The Ignition Libraries require the Xcode 10 compiler on MacOS Mojave.

On Mac machines, gcc is acquired by installing Xcode command line tools.
The required version of Xcode for Citadel is Xcode 10.3, which can be downloaded from
[Apple Developer Site](https://developer.apple.com/download/more/).
You will need to sign in to your Apple account and download the Mojave version of
Xcode command line tools. Command line tools can also be obtained by downloading
Xcode from the Apple App Store, but installing the full app may take over an hour.

This is the end of the MacOS-specific source installation instructions. Continue
to the section "Building the Ignition Libraries" to complete the installation.

## Building the Ignition Libraries

Once the compiler and all the sources are in place it is time to compile them.
Start the procedure by changing into the workspace and listing the packages
recognized by `colcon`:

```bash
cd ~/workspace/
colcon graph
```

`colcon graph` should list the Ignition libraries with an
[interdependency diagram](https://colcon.readthedocs.io/en/released/reference/verb/graph.html#example-output).
If the command works, then you are ready
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

This is the end of the source install instructions; head back to the [Getting started](getting_started.html)
page to start using Ignition!

## Uninstalling source-based install

If you need to uninstall Ignition or switch to a binary-based install once you
have already installed the library from source, navigate to your source code
directory's build folders and run `make uninstall`:

```bash
cd /workspace
sudo make uninstall
```

## Troubleshooting

### Unable to create the rendering window

If you're getting errors like "Unable to create the rendering window", it could
mean you're using an old OpenGL version. Ignition Gazebo uses the Ogre 2
rendering engine by default, which requires an OpenGL version higher than 3.3.

This can be confirmed by checking the Ogre 2 logs at `~/.ignition/rendering/ogre2.log`,
which should have an error like:

"OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported. Please update your graphics card drivers."

You can also check your OpenGL version running:

    glxinfo | grep "OpenGL version"

You should be able to use Ogre 1 without any issues however. You can check if
that's working by running a world which uses Ogre 1 instead of Ogre 2, such as:

    ign gazebo -v 3 lights.sdf

If that loads, you can continue to use Ignition with Ogre 1, just be sure to
specify `ogre` in your SDF files instead of `ogre2`.

To enable Ogre 2 support, you'll need to update your computer's OpenGL version.
As suggested on the Ogre logs, this may require updating your graphics card
drivers.

### MacOS

#### Unable to find `urdf_model.h` error

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

#### Unable to load .dylib file

When running `ign gazebo -s` command, an error like the one below may show up

```bash
Error while loading the library [/Users/citadel/citadel_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib]: dlopen(/Users/citadel/citadel_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib, 5): Library not loaded: @rpath/libIrrXML.dylib
  Referenced from: /usr/local/opt/assimp/lib/libassimp.5.dylib
  Reason: image not found
[Err] [Physics.cc:275] Unable to load the /Users/citadel/citadel_ws/install/lib//libignition-physics2-dartsim-plugin.2.dylib library.
Escalating to SIGKILL on [Ignition Gazebo Server]
```

The issue is related to OSX System Integrity Protection(SIP). The workaround is to run `ign` with a different ruby then make sure that ruby is loaded.

```bash
brew install ruby

# Add the following to ~/.bashrc
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# Source ~/.bashrc in terminal
. ~/.bashrc
```
