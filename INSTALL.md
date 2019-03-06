# Acropolis Installation

Acropolis supports the following plaforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS HighSierra

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this moment. The ign-gazebo
package is still not available for Windows, the installation should be done from
source code.

# Installation on Ubuntu Bionic



# Source code installation

To help with the source compilation the use of some tools is recommended although
other ways of correctly getting the sources and build in order the libraries are
also possible.

## Installing vcstool and colcon

For getting the sources of all libraries the easiest way is to use
[vcstool](https://github.com/dirk-thomas/vcstool). The tool is available from pip
in all platforms:

> pip install vcstool

To compile all the different libraries and ignition-gazebo in the right order
it is recommended to use the tool [colcon](https://colcon.readthedocs.io/en/released/).
The tool is available in all platforms using pip:

> pip install -U colcon-common-extensions

### Use .deb packages in Ubuntu to install vcstool and colcon

As an alternative method there are .deb packages available for Debian or Ubuntu:

> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
> sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
> sudo apt-get update
> sudo apt-get install python3-vcstool python3-colcon-common-extensions

## Getting the sources

The instructions bellow use some UNIX commands but the obvious alternatives
on Windows should provide the same result.

The first step would be to create a developer workspace in which vcstool and
colcon can work.

> mkdir -p ~/workspace/src
> cd ~/workspace/src

All the sources of ignition-acropolis are declared in a yaml file. Download
it to the workspace.

> wget https://bitbucket.org/osrf/gazebodistro/src/default/collection-acropolis.yml

Use vcstool to automatically retrieve all the ignition libraries sources from
their repositories:

> vcstool import < collection-acropolis.yml

The src subdirectory should contain all the sources ready to be built.

## Building the ignition libraries

Once all the sources are in place it is time of compiling them. Start the
procedure by locating into the workspace and listing the packages recognized
by colcon:

> cd ~/workspace/
> colcon list -g

colcon should list all the ignition family in the output with their
interdependecies. If that is the case, all is ready to 
to build the whole set of libraries:

> colcon build

If there are no errors, all the binaries should be ready to use.

## Using the workspace

The workspace binaries are ready but evertime that ign-gazebo needs to be
executed or third party code is going to be developed using the ignition
libraries, one command is needed:

> . install/local_setup.bash (or call install/local_setup.bat on Windows)

After running the command all paths for running apps or developing code 
will be set in the current shell.
