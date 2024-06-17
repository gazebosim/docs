# Source Installation on macOS

This tutorial will work for macOS BigSur 10.16 and macOS Monterey 12.0.

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and gz-sim in the right order
[colcon](https://colcon.readthedocs.io/en/released/) is recommended.
The colcon tool is available on all platforms using pip (or pip3, if pip fails).

## Python3 from homebrew

Tools and dependencies for Ionic can be installed using the [homebrew package manager](https://brew.sh/).
The homebrew tool can be installed by entering the following in a terminal:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

Ionic is compatible with Python3; it can be installed by running the following in a terminal:

```bash
brew install python3
```

## vcstool and colcon from pip

PIP is available on all platforms:

```bash
python3 -m pip install vcstool
```

```bash
python3 -m pip install -U colcon-common-extensions
```

## Getting the sources

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

Add `osrf/simulation` to Homebrew formulae

```bash
brew update
brew tap osrf/simulation
```

Install all dependencies:

Dependency for Ogre:

```bash
brew cask install xquartz
```

General dependencies:

```bash
brew install assimp boost bullet cmake cppzmq dartsim@6.10.0 doxygen eigen fcl ffmpeg flann freeimage freetype gdal gflags google-benchmark gts ipopt jsoncpp libccd libyaml libzzip libzip nlopt ode open-scene-graph ossp-uuid ogre1.9 ogre2.3 pkg-config protobuf qt@5 qwt-qt5 rapidjson ruby tbb tinyxml tinyxml2 urdfdom zeromq
```

`dartsim@6.10.0` and `qt@5` are not sym-linked. To use those dependencies when building
`gz-physics8` and `gz-gui9`, run the following after installation:

For Macs with Intel processors, add them to `/usr/local`:

```bash
# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/usr/local/opt/dartsim@6.10.0/lib:/usr/local/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/usr/local/opt/dartsim@6.10.0/lib/pkgconfig
# qt@5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/local/opt/qt@5
```

Note if you are on an ARM based Apple Silicon Mac machine, you will need to add them to `/opt/homebrew` instead:

```bash
# dartsim@6.10.0
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/homebrew/opt/dartsim@6.10.0
export DYLD_FALLBACK_LIBRARY_PATH=${DYLD_FALLBACK_LIBRARY_PATH}:/opt/homoebrew/opt/dartsim@6.10.0/lib:/opt/homebrew/opt/octomap/local
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/opt/homebrew/opt/dartsim@6.10.0/lib/pkgconfig
# qt@5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/homebrew/opt/qt@5
```

### Install compiler requirements

The Gazebo Libraries require the Xcode 10 compiler on MacOS Mojave.

On Mac machines, gcc is acquired by installing Xcode command line tools.
The required version of Xcode for Ionic is Xcode 10.3, which can be downloaded from
[Apple Developer Site](https://developer.apple.com/download/more/).
You will need to sign in to your Apple account and download the Mojave version of
Xcode command line tools. Command line tools can also be obtained by downloading
Xcode from the Apple App Store (installing the full app may take over an hour).

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

Note if you are on an ARM based Apple Silicon Mac machine, you may need to set a couple more cmake args:

```bash
colcon build --cmake-args -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib --merge-install
```

To speed up the build process, you could also disable tests by using

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

To use debuggers activate debug symbols. Gazebo will run slower, but you'll be able to use `lldb`:

```bash
colcon build --cmake-args ' -DBUILD_TESTING=OFF' ' -DCMAKE_BUILD_TYPE=Debug' --merge-install
```

To build a specific package with all its dependent packages:

```bash
colcon build --merge-install --packages-up-to PACKAGE_NAME
```

To build a single package:

```bash
colcon build --merge-install --packages-select PACKAGE_NAME
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

You should now be able to launch gazebo:

```sh
# launch server in one terminal
gz sim -v 4 shapes.sdf -s

# launch gui in a separate terminal
# remember to source the workspace setup script
gz sim -v 4 -g
```

This is the end of the source install instructions; head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!

## Troubleshooting

See [Troubleshooting](/docs/ionic/troubleshooting#macos)
