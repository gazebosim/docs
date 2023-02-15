# Introduction

This document provides an overview about the options to use different versions
of ROS in combination with different versions of Gz. It is recommended to read
it before installing the [ros_gz bridge](https://github.com/gazebosim/ros_gz).

# Important! Simple analysis for a quick and correct decision

If you are planning on using a specific version of ROS and don't have a reason
to use a specific version of Gazebo, you should proceed with the default
version of Gazebo available from the ROS repository when installing the ros_gz
bridge:

```
apt-get install ros-${ROS_DISTRO}-ros-gz

```

This command is going to install the Gazebo libraries corresponding to the
version detailed below in the section: "Using the default Gazebo version
for a ROS distribution using binary installations".

# Gazebo versions and ROS integration

Gazebo is an independent project like boost, ogre or any other project used by
ROS. Usually, the latest version of Gazebo available at the beginning of every
ROS release cycle (for example Gazebo Fortress for ROS2 Humble) is selected as
the official one to be fully integrated and supported and will be kept during
the whole life of the ROS distribution.

Note that all Gz libraries policy follows the [semantic
versioning](http://semver.org/) philosophy for ABI stability, in which all
versions that have the same major number (`gz-sim7_7.0.0`, `gz-sim7_7.1.0`,
`gz-sim7_7.0.1`, ...) are binary compatible and thus interchangeable when using
the same ROS distro.

## Installing Gazebo

### Gz simulator Ubuntu packages

The easiest way of installing Gazebo is to use packages. There are two main
repositories which host Gz simulator and Gz libraries: one is `packages.ros.org` and the
other is `packages.osrfoundation.org`. At the time of writing:

 * ***packages.ros.org***
  * ROS1 Noetic: Gz Citadel
  * ROS2 Foxy: Gz Citadel
  * ROS2 Galactic: Gz Edifice
  * ROS2 Humble: Gz Fortress
  * ROS2 Rolling: Gz Fortress (changing frequently)
 * ***packages.osrfoundation.org***
  * Gz Citadel
  * Gz Edifice
  * Gz Fortress
  * Gz Garden

This means that including the osrfoundation repository is not strictly needed
to get the Gazebo binary package in Ubuntu. It can be installed from the ROS
repository.

## Using the default Gazebo version for a ROS distribution using :binary installations

For the users that need to run a specific version of ROS and want to use all
the gazebo ROS related packages out-of-the-box, this is the recommended
section:

### ROS2 Humble and ROS2 Rolling
ROS2 Humble and ROS2 Rolling support officially the Gazebo Fortress version.
For a fully-integrated ROS system, Fortress is the recommened version. The way
to proceed is just to use the ROS repository and do ***not*** use the
osrfoundation repository.

ROS packages `ros-humble-ros-gz` and `ros-rolling-ros-gz` will be available
pulling the necessary Gazebo Fortress dependencies.

### ROS2 Galactic
ROS2 Galactic supports officially the Gazebo Edifice
version. For a fully-integrated ROS system, Edifice is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-galactic-ros-gz` will be available pulling the necessary
Gazebo Edifice dependencies.

### ROS2 Foxy
ROS2 Foxy supports officially the Gazebo Citadel
version. For a fully-integrated ROS system, Citadel is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-foxy-ros-gz` will be available pulling the necessary
Gazebo Citadel dependencies.

### ROS1 Noetic
ROS1 Noetic supports officially the Gazebo Citadel
version. For a fully-integrated ROS system, Citadel is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-noetic-ros-gz` will be available pulling the necessary
Gazebo Citadel dependencies.

## Using an specific Gazebo version with ROS2
***Warning!: Using this option, you won't be able to use any ROS Ubuntu package
related to Gz libraries from ROS deb repository.

Selecting a different release of Gz than the one officially supported in each
ROS distribution will require to compile the [`ros_gz
bridge`](https://github.com/gazebosim/ros_gz) from source. Same applies for
other ROS packages using Gz libraries that could also require code changes in
they are not supporting the new Gz version chosen.

### Gz Garden

Gz Garden can be used with ROS2 Humble and ROS2 Rolling, the [`ros_gz
bridge`](https://github.com/gazebosim/ros_gz) supports the compilations from
source.

 * Folow [the instruction to install gz-garden](https://gazebosim.org/docs/garden/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile the ros_gz bridge from source in a colcon workspace
   * [ROS2 Humble](https://github.com/gazebosim/ros_gz/tree/humble#from-source).
     * Be sure of using `export GZ_VERSION=humble`
   * [ROS2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=rolling`

### Gz Fortress

Gz Fortress can be also used with ROS2 Galactic and ROS1 Noetic, the [`ros_gz
bridge`](https://github.com/gazebosim/ros_gz) supports the compilation from
source.

 * Folow [the instruction to install gz-fortress](https://gazebosim.org/docs/fortress/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile the ros_gz bridge from source in a colcon workspace
   * [ROS2 Galactic](https://github.com/gazebosim/ros_gz/tree/galactic#from-source).
     * Be sure of using `export GZ_VERSION=fortress`
   * [ROS1 Noetic]()https://github.com/gazebosim/ros_gz/tree/noetic#from-source
     * Be sure of using `export GZ_VERSION=fortress`

### Gz Edifice
Gz Edifice can be also used with ROS2 Foxy, ROS2 Rolling and ROS1 Noetic, the [`ros_gz
bridge`](https://github.com/gazebosim/ros_gz) supports the compilation from
source.

 * Folow [the instruction to install gz-edifice](https://gazebosim.org/docs/edifice/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile the ros_gz bridge from source in a colcon workspace
   * [ROS2 Foxy](https://github.com/gazebosim/ros_gz/tree/foxy#from-source).
     * Be sure of using `export GZ_VERSION=foxy`
   * [ROS2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=rolling`
   * [ROS1 Noetic]()https://github.com/gazebosim/ros_gz/tree/noetic#from-source
     * Be sure of using `export GZ_VERSION=`edifice`

## Using the latest Gz binary versions

The Gz team usually backport and release new versions of each of the supported
Gz releases and libraries (i.e: bumping gz-sim 7.0.0 to gz-sim 7.1.0). These
updates are hosted first in the osrfoundation repository. The ROS repository
syncs from the osrfoundation repository frequently but versions can be
different.

Getting the latest versions of the Gazebo libraries and simulator is as easy
as installing the [osrfoundation.org repository](https://gazebosim.org/docs/latest/install_ubuntu_src#install-dependencies)
together with the ROS repository. Updates should be fully compatible.

## FAQ

#### I am not using ROS at all, which version should I use?

If you don't need ROS support, the recommended version is the latest released
version that can be [installed using the osrfoundation repo](https://gazebosim.org/docs)
depending on your platform.

#### I want to use Gz Garden with ROS. Where are the packages?

Unfortunately there are no Gz Garden packages for any ROS distribution. Please
read how to compile it from source in this document under the section
"Using an specific Gazebo version with ROS2" >> "Gz Garden"

#### Where I can find the different features implemented on each Gazebo version?

Some notes are regularly posted on the [Gazebo community
site](https://community.gazebosim.org/tags/c/release-announcements-and-discussions/10/release)
and special posts and videos are also posted there when a new release is out:
See the one for [Garden](https://community.gazebosim.org/t/gazebo-garden-release/1627) or the
one for [Fortress](https://community.gazebosim.org/t/ignition-fortress-release/1127) as
examples.

Additionally, navigating through the different releases of Gazebo in the top
right corner of the [Gz documentation page](https://gazebosim.org/docs) there
is a left menu entry called "Feature Comparison" that provides comparison
against gazebo-classic features.
