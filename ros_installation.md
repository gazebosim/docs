# Install Gazebo and ROS

This document provides an overview about the options to use different versions
of ROS in combination with different versions of Gazebo. It is recommended to read
it before installing [ros_gz](https://github.com/gazebosim/ros_gz).

# Important! Simple analysis for a quick and correct decision

If you are planning on using a specific version of ROS and don't have a reason
to use a specific version of Gazebo, you should proceed with the default
version of Gazebo available from the ROS repository when installing `ros_gz`:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

This command is going to install the Gazebo libraries corresponding to the
version detailed below in the section: "Using the default Gazebo version
for a ROS distribution using binary installations". The rest of this
document provides more information and other possible installations but
at this point ROS and Gazebo are installed and ready.

# Gazebo versions and ROS integration

Gazebo is an independent project like Boost, Ogre or any other project used by
ROS. Usually, the latest version of Gazebo is available at the beginning of every
ROS release cycle (for example Gazebo Fortress for ROS 2 Humble) is selected as
the official one to be fully integrated and supported and will be kept during
the whole life of the ROS distribution.

Note that all Gazebo libraries policy follows the [semantic
versioning](http://semver.org/) philosophy for ABI stability, in which all
versions that have the same major number (`gz-sim7_7.0.0`, `gz-sim7_7.1.0`,
`gz-sim7_7.0.1`, ...) are binary compatible and thus interchangeable when using
the same ROS distro.

## Installing Gazebo

### Gazebo simulator Ubuntu packages

The easiest way of installing Gazebo is to use binary packages. There are two main
repositories which host Gazebo simulator and Gazebo libraries: one is `packages.ros.org` and the
other is `packages.osrfoundation.org`. At the time of writing:

 * **packages.ros.org**
   * ROS1 Noetic: Gazebo Citadel
   * ROS2 Foxy: Gazebo Citadel
   * ROS2 Galactic: Gazebo Edifice
   * ROS2 Humble: Gazebo Fortress
   * ROS2 Rolling: Gazebo Fortress (changing frequently)

 * **packages.osrfoundation.org**
   * Gazebo Citadel
   * Gazebo Edifice
   * Gazebo Fortress
   * Gazebo Garden

This means that including the osrfoundation repository is not strictly needed
to get the Gazebo binary packages it can be installed from the ROS
repository.

## Installing the default Gazebo version for a ROS distribution using binary installations

For users that need to run a specific version of ROS and want to use all
the gazebo ROS related packages out-of-the-box, this is the recommended
section:

### ROS2 Humble and ROS2 Rolling
ROS 2 Humble and ROS 2 Rolling support officially the Gazebo Fortress version.
For a fully-integrated ROS system, Fortress is the recommened version. The way
to proceed is just to use the ROS repository and do ***not*** use the
osrfoundation repository.

ROS packages `ros-humble-ros-gz` and `ros-rolling-ros-gz` will be available
, these packages automatically install the necessary dependencies for Gazebo Fortress.

### ROS 2 Galactic 
ROS 2 Galactic officially supports Gazebo Edifice
For a fully-integrated ROS system, Edifice is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-galactic-ros-gz` will be available pulling the necessary
Gazebo Edifice dependencies.

### ROS 2 Foxy
ROS 2 Foxy supports officially the Gazebo Citadel
version. For a fully-integrated ROS system, Citadel is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-foxy-ros-gz` will be available pulling the necessary
Gazebo Citadel dependencies.

### ROS 1 Noetic
ROS Noetic officially supports Gazebo Citadel
version. For a fully-integrated ROS system, Citadel is the recommened
version. The way to proceed is just to use the ROS repository and do ***not***
use the osrfoundation repository.

ROS packages `ros-noetic-ros-gz` will be available pulling the necessary
Gazebo Citadel dependencies.

## Using an specific Gazebo version with ROS 2
<div class="warning">
<strong>Warning:</strong> Using this option, you won't be able to use any ROS Ubuntu package
related to Gazebo libraries from ROS deb repository.
</div>

Selecting a different release of Gazebo than the one officially supported in each
ROS distribution will require that you manually compile 
[`ros_gz`](https://github.com/gazebosim/ros_gz) from source. Same applies for
other ROS packages using Gazebo libraries that could also require code changes if
they are not supporting the new Gazebo version chosen.

### Gazebo Garden

Gazebo Garden can be used with ROS 2 Humble and ROS 2 Rolling, 
[`ros_gz`](https://github.com/gazebosim/ros_gz) supports the compilations from
source.

 * Folow [the instruction to install gz-garden](https://gazebosim.org/docs/garden/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Install [rosdep rules for Gazebo Garden](https://github.com/osrf/osrf-rosdep#installing-rosdep-rules-to-resolve-gazebo-garden-libraries)
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Humble](https://github.com/gazebosim/ros_gz/tree/humble#from-source)
     * Be sure of using `export GZ_VERSION=garden`
   * [ROS 2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=garden`

### Gazebo Fortress

Gazebo Fortress can be also used with ROS 2 Galactic and ROS 1 Noetic, 
[`ros_gz`](https://github.com/gazebosim/ros_gz) supports the compilation from
source.

 * Follow [these instruction to install gz-fortress](https://gazebosim.org/docs/fortress/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Galactic](https://github.com/gazebosim/ros_gz/tree/galactic#from-source)
     * Be sure of using `export GZ_VERSION=fortress`
   * [ROS1 Noetic](https://github.com/gazebosim/ros_gz/tree/noetic#from-source)
     * Be sure of using `export GZ_VERSION=fortress`

### Gazebo Edifice
Gazebo Edifice can be also used with ROS2 Foxy, ROS2 Rolling and ROS1 Noetic, 
[`ros_gz`](https://github.com/gazebosim/ros_gz) supports the compilation from 
source.

 * Folow [the instruction to install gz-edifice](https://gazebosim.org/docs/edifice/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Foxy](https://github.com/gazebosim/ros_gz/tree/foxy#from-source)
     * Be sure of using `export GZ_VERSION=edifice`
   * [ROS 2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=edifice`
   * [ROS1 Noetic](https://github.com/gazebosim/ros_gz/tree/noetic#from-source)
     * Be sure of using `export GZ_VERSION=edifice`

## Using the latest Gazebo binary versions

The Gazebo team usually backport and release new versions of each of the supported
Gazebo releases and libraries (i.e: bumping `gz-sim 7.0.0` to gz-sim `7.1.0`). These
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

#### I want to use Gazebo Garden with ROS. Where are the packages?

Unfortunately there are no Gazebo Garden packages for any ROS distribution. Please
read how to compile it from source in this document under the section
"Using an specific Gazebo version with ROS2" >> "Gazebo Garden"

#### Where I can find the different features implemented on each Gazebo version?

The best place is the Gazebo web page that hosts a list of the
[main features available in each Gazebo distribution](https://gazebosim.org/docs/latest/release-features).

Some notes are regularly posted on the [Gazebo community
site](https://community.gazebosim.org/tags/c/release-announcements-and-discussions/10/release)
and special posts and videos are also posted there when a new release is out:
See the one for [Garden](https://community.gazebosim.org/t/gazebo-garden-release/1627) or the
one for [Fortress](https://community.gazebosim.org/t/ignition-fortress-release/1127) as
examples.

Additionally, navigating through the different releases of Gazebo in the top
right corner of the [Gazebo documentation page](https://gazebosim.org/docs) there
is a left menu entry called "Feature Comparison" that provides comparison
against gazebo-classic features.
