# Ignition Roadmap

This page describes planned work for Ignition. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2022 Q1 (Jan - Mar) / Q2 (Apr - Jun) - ongoing

* **Out-of-box experience**: Improve end-user experience
    * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22OOBE+%F0%9F%93%A6%E2%9C%A8%22&state=open&type=Issues)
    * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22oobe+%F0%9F%93%A6%E2%9C%A8%22)

## Feature Roadmap

A number of features are planned for specific releases of Ignition. The
features listed here derive from the Quarterly Roadmap, listed above, and other
ongoing projects.  See the [Release Features](/docs/all/release-features) page
for a list of features already available in each release of Ignition.

A named release of Ignition, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Our
[development and release pattern](/docs/all/releases) allows us to distribute
patch and minor updates into a stable Ignition version. For example, if a new
feature does not break API/ABI then we will target the feature to the oldest
compatible non-EOL release and propagate the feature forward.

### Citadel

1. [Integration with ros2_control.](https://github.com/ignitionrobotics/ign_ros2_control/)
1. [Quick start dialog.](https://github.com/ignitionrobotics/ign-gazebo/issues/1252)
1. [Splash screen.](https://github.com/ignitionrobotics/ign-gui/issues/336)
1. [System inspector.](https://github.com/ignitionrobotics/ign-gazebo/issues/191)
1. [Notification snackbar](https://github.com/ignitionrobotics/ign-gui/issues/44)
1. [Bash completion](https://github.com/ignitionrobotics/ign-tools/issues/1)

### Edifice

1. [Improved Windows support.](https://github.com/search?q=org%3Aignitionrobotics+label%3AWindows&state=open&type=Issues)
1. [Wheel slip commands.](https://github.com/ignitionrobotics/ign-gazebo/pull/1241)

### Fortress

1. [Run server and client in the same process.](https://github.com/ignitionrobotics/ign-gazebo/pull/793)
1. [Command line: use binaries instead of shared libraries](https://github.com/ignitionrobotics/ign-tools/issues/7)
1. [Bounding box camera.](https://github.com/ignitionrobotics/ign-sensors/issues/135)
1. [GUI model editor](https://github.com/ignitionrobotics/ign-gazebo/labels/editor)
1. [Bazel build files.](https://github.com/ignitionrobotics/ign-bazel)
1. [Python interface to Gazebo.](https://github.com/ignitionrobotics/ign-gazebo/issues/789)
1. [Waves and hydrodynamics for water surface vehicles.](https://github.com/ignitionrobotics/ign-gazebo/issues/1247)
1. [Custom shaders,](https://github.com/ignitionrobotics/ign-gazebo/issues/657)
1. [Visual plugins.](https://github.com/ignitionrobotics/ign-gazebo/issues/265)
1. [Custom rendering sensors.](https://github.com/ignitionrobotics/ign-gazebo/pull/1268)
1. [USD importer / exporter.](https://github.com/ignitionrobotics/sdformat/pull/736)
1. [Omniverse](https://developer.nvidia.com/nvidia-omniverse-platform) application.
1. [Save more changed components to SDF.](https://github.com/ignitionrobotics/ign-gazebo/issues/1312)

### Garden

1. [Camera distortion.](https://github.com/ignitionrobotics/ign-sensors/issues/107)
1. [Wide angle camera.](https://github.com/ignitionrobotics/ign-sensors/issues/24)
1. [Point cloud visualization.](https://github.com/ignitionrobotics/ign-gazebo/issues/1156)
1. [Force/torque visualization.](https://github.com/ignitionrobotics/ign-gazebo/issues/1155)
1. [Reset API.](https://github.com/ignitionrobotics/ign-gazebo/issues/1107)
1. [DEM heightmaps.](https://github.com/ignitionrobotics/ign-gazebo/issues/235)
1. [Triggered cameras.](https://github.com/ignitionrobotics/ign-sensors/issues/185)
1. [Parameters in Ignition Transport.](https://github.com/ignitionrobotics/ign-gazebo/pull/1280)
1. [Bridge Ignition services to ROS 2 services.](https://github.com/ignitionrobotics/ros_ign/pull/211)
1. [Satisfying ASAN for Ignition Math.](https://github.com/ignitionrobotics/ign-math/issues/370)

## Planned releases

Please see the [Releases](/docs/all/releases) for the timeline of and information about future distributions.

## Contributing to Ignition

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](/docs/all/contributing) guide.
1. [Feature comparision](/docs/citadel/comparison) list. This page lists the
   feature gaps between Gazebo classic and Ignition Gazebo.
1. Take a look at the various [libraries](/libs), and the issue tracker
   associated with each.
