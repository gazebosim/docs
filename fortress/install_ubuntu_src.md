# Source Installation on Ubuntu

These instructions apply to Ubuntu Bionic (18.04), Focal (20.04) and Jammy (22.04).

## Install tools

The use of some additional tools is recommended to help with the source compilation,
although other ways of correctly getting and building the sources are also possible.

The easiest way to get the sources of all libraries is to use
[vcstool](https://github.com/dirk-thomas/vcstool).

To compile all the different libraries and ign-gazebo in the right order
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

Ignition libraries use `git` for version control, so it must be available
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

All the sources of ignition-fortress are declared in a yaml file. Download
it to the workspace:

```bash
curl -O https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
```

Use `vcstool` to automatically retrieve all the Ignition libraries sources from
their repositories:

```bash
vcs import < collection-fortress.yaml
```

The src subdirectory should contain all the sources ready to be built.

## Install dependencies

Before compiling it is necessary to install all the dependencies of the different
packages that compose the Fortress collection. Every platform has a different
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
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
```

### Install compiler requirements

The Ignition Libraries require the gcc compiler version 8 or higher.
(Windows requires Visual Studio 2019).

#### Ubuntu Bionic

Ubuntu Bionic's default compiler version is not high enough, so the following
steps are needed to upgrade. These are not needed on Ubuntu Focal or Jammy.

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

If there are no errors, all the binaries should be ready to use. You can check the [Troubleshooting] section for errors.

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

See [Troubleshooting](/docs/fortress/troubleshooting#ubuntu)
