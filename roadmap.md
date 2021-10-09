# Ignition Roadmap

This page describes planned work for Ignition. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2021 Q3 (Jul - Sep) / Q4 (Oct - Dec) - ongoing

* **Scripting**: Add scripting interfaces to Ignition
    * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22scripting%22&state=open&type=Issues)
    * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22scripting%22)

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

1. [GPS sensor.](https://github.com/ignitionrobotics/ign-sensors/issues/23)
1. Fuel asset version control.
1. [Schematic view widget.](https://github.com/ignitionrobotics/ign-gazebo/issues/163)
1. [Integration with ros2_control.](https://github.com/ignitionrobotics/ign_ros2_control/)
1. [Python interface for Ignition Math.](https://github.com/ignitionrobotics/ign-math/issues/210)
1. [Ray tracing rendering engine: LuxCore](https://github.com/ignitionrobotics/ign-rendering/pull/372)

### Dome

1. Localized wind (wind that is constrained to a region of influence).
1. [Bazel build files.](https://github.com/ignitionrobotics/ign-bazel)
1. [Scaling widget.](https://github.com/ignitionrobotics/ign-gazebo/issues/195)

### Edifice

1. Improved Windows support.
1. Mesh level of detail support.
1. Design for Enhanced distributed simulation.

### Fortress

1. [Run server and client in the same process.](https://github.com/ignitionrobotics/ign-gazebo/pull/793)
1. [Command line: use binaries instead of shared libraries](https://github.com/ignitionrobotics/ign-tools/issues/7)
1. Sensors
  1. [Bounding box camera.](https://github.com/ignitionrobotics/ign-sensors/issues/135)
1. GUI features
    1. [Model editor](https://github.com/ignitionrobotics/ign-gazebo/labels/editor)

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
