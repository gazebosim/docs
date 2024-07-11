# ROS 2 Gazebo Vendor Packages

## History of Gazebo packaging before ROS 2 Jazzy

ROS 2 versions prior to Jazzy used Gazebo packages that were available in
upstream Ubuntu. However, due to the package
[update policy of Ubuntu](https://wiki.ubuntu.com/StableReleaseUpdates), the
Gazebo packages on upstream Ubuntu did not receive any updates. Thus, upstream
Ubuntu almost always had older versions of Gazebo than what was available from
the Gazebo package repository ([packages.osrfoundation.org](packages.osrfoundation.org)). To overcome this,
more up-to-date Gazebo packages were copied to the ROS bootstrap repository
([repos.ros.org](repos.ros.org)) which were then copied to the main ROS package repository
([packages.ros.org](packages.ros.org)). However, the process was error prone and keeping package
versions in sync between the Gazebo and ROS package repositories was difficult
since this was a manual process.

## Gazebo vendor packages

As of ROS 2 Jazzy, Gazebo is available from the ROS package repository via
vendor packages. A ROS vendor package is a ROS package that provides software
that ROS needs on platforms where it might not be available, or where a
different version than what is available is required [^1].

The Gazebo vendor packages provide all of the Gazebo libraries to a given ROS
release. The packages are built in the ROS buildfarm and as part of their build
process, fetch the sources of the underlying Gazebo library and build it. In
addition, the vendor packages provide CMake shims that make it possible to use
CMake targets without version numbers.

Each release of ROS 2 is paired with a specific release of Gazebo. For ROS 2
Jazzy, the vendor packages contain Gazebo libraries from the Harmonic release.
For ROS 2 K-turtle (next release), the vendor packages will contain Gazebo
Ionic. This default pairing is found in the
[ROS installation instructions](https://gazebosim.org/docs/harmonic/ros_installation)

The following is a table of all vendor packages and the underlying Gazebo
library they vendor:

| Vendor Package                                                                   | Gazebo Library                                              |
| -------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| [gz_cmake_vendor](https://github.com/gazebo-release/gz_cmake_vendor)             | [gz-cmake](https://github.com/gazebosim/gz-cmake)           |
| [gz_common_vendor](https://github.com/gazebo-release/gz_common_vendor)           | [gz-common](https://github.com/gazebosim/gz-common)         |
| [gz_fuel_tools_vendor](https://github.com/gazebo-release/gz_fuel_tools_vendor)   | [gz-fuel-tools](https://github.com/gazebosim/gz-fuel-tools) |
| [gz_gui_vendor](https://github.com/gazebo-release/gz_gui_vendor)                 | [gz-gui](https://github.com/gazebosim/gz-gui)               |
| [gz_launch_vendor](https://github.com/gazebo-release/gz_launch_vendor.git)       | [gz-launch](https://github.com/gazebosim/gz-launch)         |
| [gz_math_vendor](https://github.com/gazebo-release/gz_math_vendor.git)           | [gz-math](https://github.com/gazebosim/gz-math)             |
| [gz_msgs_vendor](https://github.com/gazebo-release/gz_msgs_vendor.git)           | [gz-msgs](https://github.com/gazebosim/gz-msgs)             |
| [gz_physics_vendor](https://github.com/gazebo-release/gz_physics_vendor.git)     | [gz-physics](https://github.com/gazebosim/gz-physics)       |
| [gz_plugin_vendor](https://github.com/gazebo-release/gz_plugin_vendor.git)       | [gz-plugin](https://github.com/gazebosim/gz-plugin)         |
| [gz_rendering_vendor](https://github.com/gazebo-release/gz_rendering_vendor.git) | [gz-rendering](https://github.com/gazebosim/gz-rendering)   |
| [gz_sensors_vendor](https://github.com/gazebo-release/gz_sensors_vendor.git)     | [gz-sensors](https://github.com/gazebosim/gz-sensors)       |
| [gz_sim_vendor](https://github.com/gazebo-release/gz_sim_vendor.git)             | [gz-sim](https://github.com/gazebosim/gz-sim)               |
| [gz_tools_vendor](https://github.com/gazebo-release/gz_tools_vendor.git)         | [gz-tools](https://github.com/gazebosim/gz-tools)           |
| [gz_transport_vendor](https://github.com/gazebo-release/gz_transport_vendor.git) | [gz-transport](https://github.com/gazebosim/gz-transport)   |
| [gz_utils_vendor](https://github.com/gazebo-release/gz_utils_vendor.git)         | [gz-utils](https://github.com/gazebosim/gz-utils)           |
| [sdformat_vendor](https://github.com/gazebo-release/sdformat_vendor.git)         | [sdformat](https://github.com/gazebosim/sdformat)           |

In addition to Gazebo libraries, two dependencies of Gazebo are also vendored:

| Vendor Package                                                                   | External Library                                   |
| -------------------------------------------------------------------------------- | -------------------------------------------------- |
| [gz_dartsim_vendor](https://github.com/gazebo-release/gz_dartsim_vendor.git)     | [dartsim](https://github.com/dartsim/dart)         |
| [gz_ogre_next_vendor](https://github.com/gazebo-release/gz_ogre_next_vendor.git) | [ogre-next](https://github.com/OGRECave/ogre-next) |

**Note:** These two vendor packages are only intended to be consumed by Gazebo.
Use of this vendor package generally (outside of Gazebo) is not recommended as
the underlying library version might change without notice.

## Running Gazebo from vendor packages

To be able to use the `gz` command to run the usual commands, be sure that at least
`gz_tools_vendor` package is installed. To have the `gz` command in the PATH,
source the `setup.bash` from `/opt/ros/${ROS_DISTRO}` as usual.

```
# Example running gz sim on Jazzy
export ROS_DISTRO=jazzy
sudo apt-get install ros-${ROS_DISTRO}-gz-tools-vendor ros-${ROS_DISTRO}-gz-sim-vendor
. /opt/ros/jazzy/setup.bash
gz sim --help
```

## Building packages using Gazebo vendor packages

### Declaring dependencies in `package.xml`

To use a Gazebo vendor package in your project, you'll need to add the
appropriate package in your `package.xml`. For example, if you use the Gazebo
libraries `gz-cmake`, `gz-utils` and `gz-math`, the entries in `package.xml`
will be the following

```xml
  ...
  <depend>gz_cmake_vendor</depend>
  <depend>gz_utils_vendor</depend>
  <depend>gz_math_vendor</depend>
  ...
```

### CMakeLists.txt for building with Gazebo vendor packages

To use a Gazebo library provided by a vendor package, you'll need to
`find_package` the vendor package and the underlying Gazebo library. Calling
`find_package` on the vendor package indicates that the project is "buying into"
the CMake shims provided by the vendor package. These shims make it possible to
`find_package` the underlying library and use its CMake targets without
including the version number in the name. Thus, when upgrading to newer versions
of Gazebo, the project `CMakeLists.txt` file would not need to be modified.

Following the example above, The CMake entry for using `gz-cmake`, `gz-utils`
and `gz-math` will be the following:

```cmake
find_package(gz_cmake_vendor REQUIRED)
find_package(gz-cmake REQUIRED)
find_package(gz_utils_vendor REQUIRED)
find_package(gz-utils REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

add_executable(test_gz_vendor main.cc)
target_link_libraries(test_gz_vendor PUBLIC gz-math::gz-math gz-utils::gz-utils)
```

**Note:** The vendor packages use underscores (`_`) while the Gazebo library
names use dashes (`-`).


## Expert use cases

The documentation provided above should cover the users that require to install
and run the Gazebo simulator together with ROS and developing code that requires
of Gazebo and ROS.

Below are listed more advanced topics for the expert users.

### Installing Non-Default Gazebo/ROS 2 Pairings with vendor packages

If you want to use a new release of Gazebo that is not officially paired with
the release of ROS you are using (e.g. Gazebo Ionic with ROS 2 Jazzy), you can
follow the following steps:

1. Install the binaries from [packages.osrfoundation.org](packages.osrfoundation.org). Make sure to install
   the metapackage that includes all library components (e.g. gz-ionic,
   gz-harmonic, etc.)
1. Clone the set of vendor packages included in
   [gz_vendor.repos](https://gist.github.com/azeey/a94adb591475ea0e613313d3540ca451)
   in your workspace using [vcstool](https://github.com/dirk-thomas/vcstool).
1. Checkout the desired branch. The branch names currently track ROS 2 releases.
   You'll need to determine the branch based on the
   [default pairings](/docs/latest/ros_installation)
1. Add any additional packages you need that also depend on Gazebo, such as
   [`ros_gz`](https://github.com/gazebosim/ros_gz) and
   [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control/)
1. `export GZ_RELAX_VERSION_MATCH=1`. By default, the vendor packages look for
   an exact version match of the vendored library to be available in the system.
   If the exact version is not found, the sources are fetched and built. This
   environment variable causes the vendor package to match just the major
   version number.
1. Build the workspace

### Gazebo library development with vendor packages

Building the underlying packages from source might be needed when using a Gazebo
release that is not officially paired with the ROS release you are using or when
developing on a Gazebo library in conjunction with another ROS package that uses
Gazebo directly. You can follow these steps to build the vendor packages from
source:

1. Clone the desired release of the Gazebo libraries into a workspace. You can
   use a collection file from <https://github.com/gazebo-tooling/gazebodistro>
   (e.g.
   [`collection-harmonic.yaml`](https://github.com/gazebo-tooling/gazebodistro/blob/master/collection-harmonic.yaml))
   with [vcstool](https://github.com/dirk-thomas/vcstool).
1. Clone the set of vendor packages included in
   [gz_vendor.repos](https://gist.github.com/azeey/a94adb591475ea0e613313d3540ca451)
   in the same workspace using
   [vcstool](https://github.com/dirk-thomas/vcstool).
1. Checkout the desired branch. The branch names currently track ROS 2 releases.
   You'll need to determine the branch based on the
   [official pairings](/docs/latest/ros_installation)
1. Add any additional packages you need that also depend on Gazebo, such as
   [`ros_gz`](https://github.com/gazebosim/ros_gz) and
   [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control/)
1. `export GZ_BUILD_FROM_SOURCE=1`. By default, the vendor packages look for an
   exact version match of the vendored library to be available in the system. If
   the exact version is not found, the sources are fetched and built. Also, by
   default, the vendor packages do not depend directly on the Gazebo libraries
   you have in your workspace. Setting this environment variable causes the
   vendor package to match just the major version number and add dependencies on
   the Gazebo libraries being built from source so that the correct build order
   is followed.
1. Build the workspace

## Known Limitations

- The vendor packages are currently built with Python bindings disabled
  ([issue](https://github.com/gazebo-tooling/gz_vendor/issues/2))
- `gz_dartsim_vendor` and `gz_ogre_next_vendor` currently always build from
  source
  ([gz_dartsim_vendor issue](https://github.com/gazebo-release/gz_dartsim_vendor/issues/4),
  [gz_ogre_next_vendor issue](https://github.com/gazebo-release/gz_ogre_next_vendor/issues/4))
- When building Gazebo libraries from source, it is not possible to build some
  packages from source and install some packages from binaries.

[^1]: https://robotics.stackexchange.com/a/93262/31574
