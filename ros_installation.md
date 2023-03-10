# Installing Gazebo with ROS

This document provides guidance on using different versions
of ROS in combination with different versions of Gazebo. We recommended to read
it before installing [ros_gz](https://github.com/gazebosim/ros_gz).

# Picking the "Correct" Versions of ROS & Gazebo

If this is your first time using ROS and Gazebo, and you are not following
specific instructions or a tutorials, we recommend using the latest long term
support (LTS) versions of both ROS and Gazebo for the best user
experience. Successive versions of ROS and Gazebo releases are named
alphabetically, and the documentation for each version of ROS and Gazebo will
indicate if it is an LTS version. It is worth noting that each version of ROS
works best with a specific version of each Tier 1 platform. Tier one platforms
are platforms / host operating systems that are used in the development of ROS
and Gazebo. All of this information is outlined in
[REP-2000](https://www.ros.org/reps/rep-2000.html). To summarize, the best user
experience is to use the latest LTS version of ROS and the Tier 1 platform /
operating system recommend for that version of ROS. If you host operating system
does not match the Tier 1 operating system, consider using the Tier 1 platform
in a virtual machine. This approach has the added benefit of not modifying your
host OS and allowing you to roll back your mistakes.

At the time of writing, our recommendation is that new users install:

* [Ubuntu Jammy 22.04](https://www.releases.ubuntu.com/jammy/)
* [ROS 2 Humble Hawksbill](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027)
* [Gazebo Fortress](https://gazebosim.org/docs/fortress/install)

# Summary of Compatible ROS and Gazebo Combinations

This table includes all currently supported versions of ROS and Gazebo. All
other ROS and Gazebo releases are end of life and we do not recommend their
continued use.

|                        	| **GZ Citadel (LTS)** 	| **GZ Fortress (LTS)** 	| **GZ Garden** 	|
|------------------------	|----------------------	|-----------------------	|---------------	|
| **ROS 2 Rolling**      	| X                    	| ✔️                     	| P             	|
| **ROS 2 Humble (LTS)** 	| X                    	| ✔️                     	| P             	|
| **ROS 2 Foxy (LTS)**   	| ✔️                    	| X                     	| X             	|
| **ROS 1 Noetic (LTS)** 	| ✔️                    	| P                     	| X             	|


* ✔️ - Recommended combination
* X - Not recommended
* P - Possible,*but not recommended*. These combinations of ROS and Gazebo can
  be made to work together, but some effort is required.


## Installing Gazebo with Older Versions of ROS

If you are planning on using a specific version of ROS and don't have a reason
to use a specific version of Gazebo, we recommend using the default version of
Gazebo available from the ROS repository when installing `ros_gz`. The following
command will install the correct version of Gazebo and `ros_gz` for your ROS
installation on a Linux system. You should replace `${ROS_DISTRO}` with your ROS
distribution (e.g. `humble`, `rolling`,`foxy`, etc).

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

This command will install the Gazebo libraries recommended for your ROS
installation.

<div class="warning">
The rest of this document provides more information and other, alternative,
installations pathways. We do not recommend the following approaches for new
users.**

# Specific ROS and Gazebo Pairings

Gazebo is an independent project like Boost, Ogre, and many other projects used
by ROS. Usually, the latest version of Gazebo is available at the beginning of
each ROS release cycle (for example Gazebo Fortress for ROS 2 Humble). This
version of Gazebo is selected as the official Gazebo release for that ROS Distro
and is fully integrated, tested, and supported for the life of the ROS
distribution.

Note that all Gazebo libraries follow the [semantic
versioning](http://semver.org/) philosophy for ABI stability. All Gazebo
versions that have the same major number (`gz-sim7_7.0.0`, `gz-sim7_7.1.0`,
`gz-sim7_7.0.1`, ...) are binary compatible and thus interchangeable with a
given ROS distro.

## Installing Gazebo

### Gazebo Packages for Ubuntu

The easiest way of installing Gazebo on Ubuntu is to use binary packages. There
are two main repositories that host Gazebo simulator and Gazebo libraries: one
is `packages.ros.org` and the other is `packages.osrfoundation.org`. At the time
of
writing the following packages are available on the following hosts:

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
to get the Gazebo binary packages, as it can be installed from the ROS
repository.

## Installing the default Gazebo version for a ROS distribution using binary installations

For users that need to run a specific version of ROS and want to use all
the gazebo ROS related packages out-of-the-box, this is the recommended taking
the following approach

### ROS 2 Humble and ROS 2 Rolling
ROS 2 Humble and ROS 2 Rolling officially support Gazebo Fortress, meaning
Fortress is the recommended version of Gazebo to use. 

To install this combination of ROS and Gazebo first install ROS and then install
either `ros-humble-ros-gz` and `ros-rolling-ros-gz` from `packages.ros.org`.
These packages will automatically install both Gazebo Fortress and the ros-gz
bridge.

### ROS 2 Galactic 
**ROS 2 Galactic is presently end of life, and we do not recommend using this
version of ROS or Gazebo.** However, if you must this version of ROS, it officially supports
Gazebo Edifice. To install Gazebo Edifice simply install `ros-galactic-ros-gz`
from `packages.ros.org`. This package will automatically install both Gazebo
Edifice and the ros-gz bridge.

### ROS 2 Foxy
ROS 2 Foxy officially supports Gazebo Citadel, meaning the recommended version
of Gazebo for ROS 2 Foxy is Citadel.To install Gazebo Citadel with Foxy simply
install `ros-foxy-ros-gz` from `packages.ros.org`. This package will
automatically install both Gazebo Citadel and the ros-gz bridge.

### ROS 1 Noetic
ROS Noetic officially supports Gazebo Citadel, meaning the recommended version
of Gazebo for ROS Noetic is Citadel.To install Gazebo Citadel with Noetic simply
install `ros-noetic-ros-gz` from `packages.ros.org`. This package will
automatically install both Gazebo Citadel and the ros-gz bridge.

## Using a specific and unsupported Gazebo version with ROS 2
<div class="warning">
<strong>Warning:</strong> Only use this approach if you absolutely need to run
a version of Gazebo that is not officially supported by your ROS distro. Using
this approach will make it impossible to use the official ROS Ubuntu packages.

We do not recommend this approach for beginners!
</div>

To select a different release of Gazebo than the one officially supported by
your ROS distribution you must **manually compile**
[`ros_gz`](https://github.com/gazebosim/ros_gz) from source. This rule applies for
every  ROS packages that uses a Gazebo library. This approach may also require
that you modify your  ROS or Gazebo source code to support this compilation.

### Gazebo Garden (Not Recommended)

Gazebo Garden can be used with ROS 2 Humble and ROS 2 Rolling, but
[`ros_gz`](https://github.com/gazebosim/ros_gz) will need to be compiled
from source.

 * Folow [these instruction to install gz-garden](https://gazebosim.org/docs/garden/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Install [rosdep rules for Gazebo Garden](https://github.com/osrf/osrf-rosdep#installing-rosdep-rules-to-resolve-gazebo-garden-libraries)
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Humble](https://github.com/gazebosim/ros_gz/tree/humble#from-source)
    * Be sure of using `export GZ_VERSION=garden`
   * [ROS 2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=garden`

### Gazebo Fortress (Not Recommended)

Gazebo Fortress can be used with ROS 2 Galactic and ROS 1 Noetic by compiling
[`ros_gz`](https://github.com/gazebosim/ros_gz) from source
source.

 * Follow [these instruction to install gz-fortress](https://gazebosim.org/docs/fortress/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Galactic](https://github.com/gazebosim/ros_gz/tree/galactic#from-source)
     * Be sure of using `export GZ_VERSION=fortress`
   * [ROS1 Noetic](https://github.com/gazebosim/ros_gz/tree/noetic#from-source)
     * Be sure of using `export GZ_VERSION=fortress`

### Gazebo Edifice (Not Recommended)
Gazebo Edifice is currently end of life, and its use is not recommeneded,
However, Gazebo Edifice can be made to work with ROS 2 Foxy, ROS 2 Rolling and
ROS 1 Noetic, by compiling
[`ros_gz`](https://github.com/gazebosim/ros_gz) from source.

 * Folow [the instruction to install gz-edifice](https://gazebosim.org/docs/edifice/install_ubuntu#binary-installation-on-ubuntu)
   from osrfoundation.org repository
 * Follow the instructions to compile `ros_gz` from source in a colcon workspace
   * [ROS 2 Foxy](https://github.com/gazebosim/ros_gz/tree/foxy#from-source)
     * Be sure of using `export GZ_VERSION=edifice`
   * [ROS 2 Rolling](https://github.com/gazebosim/ros_gz/tree/ros2#from-source)
     * Be sure of using `export GZ_VERSION=edifice`
   * [ROS1 Noetic](https://github.com/gazebosim/ros_gz/tree/noetic#from-source)
     * Be sure of using `export GZ_VERSION=edifice`


## Using the Latest Gazebo Source Code for a Gazebo Distribution

The Gazebo team usually backports and releases new versions of each of the
supported Gazebo releases and libraries (i.e: bumping `gz-sim 7.0.0` to gz-sim
`7.1.0`). These updates are hosted first in the packages.osrfoundation.org
repository. The ROS repository syncs from the osrfoundation repository
frequently but versions can be different.

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
