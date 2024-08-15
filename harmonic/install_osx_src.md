# Source Installation on macOS

This tutorial will work for macOS BigSur 10.16 and macOS Monterey 12.0.

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and gz-sim in the right order
[colcon](https://colcon.readthedocs.io/en/released/) is recommended.

Tools and dependencies for Harmonic can be installed using the [Homebrew package manager](https://brew.sh/). After installing Homebrew, add the `osrf/simulation` to Homebrew tap to be able to install prebuilt dependencies.

```bash
brew tap osrf/simulation
brew update
```

### Install compiler requirements

Building Gazebo Libraries require at least Xcode 10 on MacOS Mojave. The Xcode Command Line Tools are sufficient, and can be installed with:

```bash
xcode-select --install
```

## Python 3 from Homebrew

Harmonic is compatible with Python 3. Install the latest version from Homebrew:

```bash
brew install python3
```

## vcstool and colcon from PyPI

```bash
python3 -m pip install -U colcon-common-extensions vcstool
```

## Getting the sources

The first step is to create a developer workspace in which `vcstool` and
`colcon` can work:

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src
```

All the sources of gazebo-harmonic are declared in a yaml file. Download
it to the workspace:

```bash
curl -OL https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml
```

Use `vcstool` to automatically retrieve all the Gazebo libraries sources from
their repositories:

```bash
vcs import < collection-harmonic.yaml
```

The src subdirectory should contain all the sources ready to be built.

## Install dependencies

Dependency for Ogre:

```bash
brew install --cask xquartz
```

General dependencies:

```bash
brew install assimp boost bullet cmake cppzmq dartsim doxygen eigen fcl ffmpeg flann freeimage freetype gdal gflags google-benchmark gts ipopt jsoncpp libccd libyaml libzzip libzip nlopt ode open-scene-graph ossp-uuid ogre1.9 ogre2.3 pkg-config protobuf qt@5 qwt-qt5 rapidjson ruby tbb tinyxml tinyxml2 urdfdom zeromq
```

`qt@5` is a "keg only" Homebrew formula and its path must be explicitly configured before building Gazebo:

```bash
# qt@5
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}`brew --prefix qt@5`
```

Unlink `qt` to avoid conflicts with other versions of qt (e.g. qt6)

```
brew unlink qt
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

Run the following command to source the workspace in zsh (the default macOS shell):

```zsh
. ~/workspace/install/setup.zsh
```

Or if you are using bash:

```bash
. ~/workspace/install/setup.bash
```

You should now be able to launch gazebo:

```sh
# launch server in one terminal
gz sim -v 4 shapes.sdf -s

# launch gui in a separate terminal
# remember to source the workspace setup script
gz sim -v 4 -g
```

This is the end of the source install instructions; head back to the [Getting started](getstarted)
page to start using Gazebo!

## Troubleshooting

See [Troubleshooting](troubleshooting.md#macos)
