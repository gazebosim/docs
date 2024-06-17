# Gazebo Roadmap

This page describes planned work for Gazebo. The set of planned
features and development efforts should provide insight into the overall
direction of Gazebo. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2023 Q4 (Oct - Dec)

* **Improve documentation website (gazebosim.org/docs)**
    * Use a static site generator for the documentation website so that anyone can preview changes made in a PR. (https://github.com/gazebosim/docs/issues/85)
    * Make doxygen tutorials from each library accessible on the main tutorials page on gazebosim.org/docs (https://github.com/gazebosim/docs/issues/55)
    * Add new tutorials
    * Copy/adapt tutorials from https://classic.gazebosim.org/tutorials
    * Improve existing content
    * Establish QA checklist for PRs in https://github.com/gazebosim/docs 
* **Migrate from Qt5 to Qt6** (https://github.com/gazebosim/gz-gui/issues/586)
* **Define platform dependencies and prepare for the new Ubuntu LTS 24.04 and the interaction with ROS 2 Jazzy.**
* **Support nested models in Bullet-featherstone gz-physics engine plugin**

## 2024 Q1 (Jan - Mar)

* **Improve documentation website (gazebosim.org/docs)** (Continued from Q4/2023)
* **Migrate from Qt5 to Qt6** (https://github.com/gazebosim/gz-gui/issues/586) (Continued from Q4/2023)
* **Define platform dependencies and prepare for the new Ubuntu LTS 24.04 and the interaction with ROS 2 Jazzy.** (Continued from Q4/2023)
* **Support nested models in Bullet-featherstone gz-physics engine plugin** (Continued from Q4/2023)
* **Align ROS and Gazebo messages where possible**
  * Make performance improvements to the `ros_gz` bridge
* **Improve performance of Rendering Sensors (e.g., Lidar, Depth Cameras)**
* **Set state of simulation in SDFormat (e.g. initial joint position and velocity)**

## 2024 Q2 (Apr - Jun)

* **Migrate from Qt5 to Qt6** (https://github.com/gazebosim/gz-gui/issues/586) (Continued from Q1/2024)
* **Complete Bullet-featherstone implementation**
* **Improve performance of Rendering Sensors (e.g., Lidar, Depth Cameras)** (Continued from Q1/2024)
* **Create a third party plugin repository**

## 2024 Q3 (July - Sep)

* **Create a third party plugin repository** (Continued from Q2/2024)
* **Implement Mimic joints for DART**
* **Prepare for release (feature freeze/code freeze)**
* **Prepare and Run tutorial party**

## Feature Roadmap

A number of features are planned for specific releases of Gazebo. The
features listed here derive from the Quarterly Roadmap, listed above, and other
ongoing projects.  See the [Release Features](/docs/all/release-features) page
for a list of features already available in each release of Gazebo.

A named release of Gazebo, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Our
[development and release pattern](/docs/all/releases) allows us to distribute
patch and minor updates into a stable Gazebo version. For example, if a new
feature does not break API/ABI then we will target the feature to the oldest
compatible non-EOL release and propagate the feature forward.

### Citadel

### Fortress

1. [Run server and client in the same process.](https://github.com/gazebosim/gz-sim/pull/793)
1. [Command line: use binaries instead of shared libraries](https://github.com/gazebosim/gz-tools/issues/7)
1. [GUI model editor](https://github.com/gazebosim/gz-sim/labels/editor)
1. [Save more changed components to SDF.](https://github.com/gazebosim/gz-sim/issues/1312)
1. [Improved Windows support.](https://github.com/search?q=org%3Agazebosim+label%3AWindows&state=open&type=Issues)
1. [Parameters in Gazebo Transport.](https://github.com/gazebosim/gz-transport/pull/305)

### Garden

1. [Satisfying ASAN for Gazebo Math.](https://github.com/gazebosim/gz-math/issues/370)
1. [SDF APIs to prevent console logging.](https://github.com/gazebosim/sdformat/issues/820)
1. [Download Fuel models on the background](https://github.com/gazebosim/gz-sim/issues/1260)
1. [Bazel build files.](https://github.com/gazebosim/gz-bazel)

### Harmonic

### Ionic

See the [Quarterly Roadmap](/docs/all/roadmap#quarterly-roadmap) above.

## Planned releases

Please see the [Releases](https://github.com/gazebosim/docs/blob/master/releases.md) for the timeline of and information about future distributions.


## Contributing to Gazebo

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](/docs/all/contributing) guide.
1. [Feature comparision](/docs/citadel/comparison) list. This page lists the
   feature gaps between Gazebo classic and Gazebo Sim.
1. Take a look at the various [libraries](/libs), and the issue tracker
   associated with each.
