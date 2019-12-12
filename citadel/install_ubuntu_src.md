# Source Installation on Ubuntu Bionic

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and ign-gazebo in the right order
[colcon](https://colcon.readthedocs.io/en/released/) is recommended.
The colcon tool is available on all platforms using pip (or pip3, if pip fails).

Some tools require Python 3.5 (or higher) which is not the default option on some
platforms (like Ubuntu Bionic). The Python
[virtualenv](https://virtualenv.pypa.io/en/latest/) could be a useful solution in
cases where the default option cannot be easily changed.

## vcstool and colcon from pip

PIP is available on all platforms:

```bash
pip install vcstool
```

```bash
pip install -U colcon-common-extensions
```

## vcstool and colcon from apt

An alternative method is to use the `.deb` packages available on Debian or Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
```

## Mercurial

Ignition libraries use `mercurial` for version control, so it must be available
in the system for `vcstool` to work properly. While `mercurial` is available via pip,
Python 3 support is currently in beta. Therefore a different means of installation
is recommended.

```bash
sudo apt-get install mercurial
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

All the sources of ignition-citadel are declared in a yaml file. Download
it to the workspace:

```bash
wget https://bitbucket.org/osrf/gazebodistro/raw/default/collection-citadel.yaml
```

Use `vcstool` to automatically retrieve all the Ignition libraries sources from
their repositories:

```bash
vcs import < collection-citadel.yaml
```

The src subdirectory should contain all the sources ready to be built.

## Install dependencies

Before compiling it is necessary to install all the dependencies of the different
packages that compose the Citadel collection. Every platform has a different
method to install software dependencies.

Add `packages.osrfoundation.org` to the apt sources list:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

The command below will install all dependencies in Ubuntu Bionic:

```bash
sudo apt-get install cmake freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libdart6-collision-ode-dev libdart6-dev libdart6-utils-urdf-dev libfreeimage-dev libgflags-dev libglew-dev libgts-dev libogre-1.9-dev libogre-2.1-dev libprotobuf-dev libprotobuf-dev libprotoc-dev libqt5core5a libswscale-dev libtinyxml2-dev libtinyxml-dev pkg-config protobuf-compiler python qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings qml-module-qtquick2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtqml-models2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev ruby ruby-ronn uuid-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libzmq3-dev libsqlite3-dev libwebsockets-dev swig ruby-dev -y
```

### Install compiler requirements

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
If that is the case, then you are ready
to build the whole set of libraries:

```bash
colcon build --merge-install
```

To speed up the build process, you could also disable tests by using

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
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

A source-based install can be "uninstalled" using several methods, depending on
the results you want:

  1. If you installed your workspace with `colcon` as instructed above, "uninstalling"
     could be just a matter of opening a new terminal and not sourcing the
     workspace's `setup.sh`. This way, your environment will behave as though
     there is no Ignition install on your system.

  2. If, in addition to not wanting to use the libraries, you're also trying to
     free up space, you can delete the entire workspace directory with:

     ```bash
     rm -rf ~/workspace
     ```

  3. If you want to keep the source code, you can remove the
     `install` / `build` / `log` directories as desired, leaving the `src` directory.

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

The Ogre 2 debs from the osrfoundation repository are built from a fork of
Ogre's `v2-1` branch with changes needed for deb packaging and allowing it to
be co-installable with Ogre 1.x. The code can be found here:

https://github.com/osrf/ogre-2.1-release

