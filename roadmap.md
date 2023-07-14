# Gazebo Roadmap

This page describes planned work for Gazebo. The set of planned
features and development efforts should provide insight into the overall
direction of Gazebo. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2023 Q1 (Jan - Mar) / Q2 (Apr - Jun) - ongoing

* **Out-of-box experience**: Improve end-user experience
    * [All open tickets](https://github.com/search?q=org%3Agazebosim+label%3A%22OOBE+%F0%9F%93%A6%E2%9C%A8%22&state=open&type=Issues)
    * [Status](https://github.com/orgs/gazebosim/projects/1?card_filter_query=label%3A%22oobe+%F0%9F%93%A6%E2%9C%A8%22)

* **APIs that ease migration from Gazebo-classic**
    * Tickets: [gz-sim#325](https://github.com/gazebosim/gz-sim/issues/325), [gz-sim#85](https://github.com/gazebosim/gz-sim/issues/85)

* **Provide infrastructure to generate bindings for gz-msg protos instead of packaging generated code**
    * Tickets: Part of [gz-sim#494](https://github.com/gazebosim/gz-sim/issues/494), Related to: [gz-sim#36](https://github.com/gazebosim/gz-msgs/issues/36) and [gz-sim#113](https://github.com/gazebosim/gz-msgs/issues/113)

* **Complete Python scripting capability with documentation and examples**
    * Goals: 
      * Ability to control simulation from python
      * Ability to write system "plugins" in python
      * Using Gazebo in a jupyter notebook
    * Tickets: Part of [gz-sim#790](https://github.com/gazebosim/gz-sim/issues/790)

## 2023 Q3 (Jul - Sept)

* **Complete Python scripting capability with documentation and examples**
    * Tickets: Part of [gz-sim#790](https://github.com/gazebosim/gz-sim/issues/790)

* **Improve interoperability with ROS**
    * Goals:
      -  Make Joint state publisher work with SDF
      -  Eliminate the need for having both URDF and SDF files in robot model packages
      -  Create more documentation around ROS 2 launch and resources (eg. meshes)
      -  Create a better workflow for creating custom message bridges other than forking the `ros_gz` repo. 
      -  Events emitted by Gazebo to indicate completion of world/model loading.

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

1. [Bash completion](https://github.com/gazebosim/gz-tools/issues/1)

### Fortress

1. [Run server and client in the same process.](https://github.com/gazebosim/gz-sim/pull/793)
1. [Command line: use binaries instead of shared libraries](https://github.com/gazebosim/gz-tools/issues/7)
1. [GUI model editor](https://github.com/gazebosim/gz-sim/labels/editor)
1. [Save more changed components to SDF.](https://github.com/gazebosim/gz-sim/issues/1312)
1. [Improved Windows support.](https://github.com/search?q=org%3Agazebosim+label%3AWindows&state=open&type=Issues)
1. [Parameters in Gazebo Transport.](https://github.com/gazebosim/gz-transport/pull/305)

### Garden

1. [Satisfying ASAN for Gazebo Math.](https://github.com/gazebosim/gz-math/issues/370)
1. [Mimic joint type.](https://github.com/gazebosim/sdf_tutorials/pull/62)
1. [SDF APIs to prevent console logging.](https://github.com/gazebosim/sdformat/issues/820)
1. [Download Fuel models on the background](https://github.com/gazebosim/gz-sim/issues/1260)
1. [Bazel build files.](https://github.com/gazebosim/gz-bazel)
1. [Dynamic wrench application through the GUI (Mouse interaction with simulated models)](https://github.com/gazebosim/gz-sim/issues/306)

### Harmonic

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
