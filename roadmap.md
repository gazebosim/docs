# Ignition Roadmap

This page describes planned work for Ignition. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2021 Q2 (Apr - Jun) - ongoing

* **Performance**: Improve Ignition Gazebo's runtime performance
    * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22performance%22&state=open&type=Issues)
    * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22performance%22)

## 2021 Q3 (Jul - Sep) / Q4 (Oct - Dec) - planned

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

### Dome

1. Localized wind (wind that is constrained to a region of influence).
1. [Bazel build files.](https://github.com/ignitionrobotics/ign-bazel)
1. [Bullet physics engine.](https://github.com/ignitionrobotics/ign-physics/issues/44)
1. [Parameter passing on SDF files.](https://github.com/osrf/sdformat/pull/413)

### Edifice

1. Improved Mac and Windows support.
1. Mesh level of detail support.
1. Design for Enhanced distributed simulation.
1. [Hook command line tool to binaries instead of libraries.](https://github.com/ignitionrobotics/ign-tools/issues/7)
1. [Heightmap support using Ogre 1 and DART.](https://github.com/ignitionrobotics/ign-gazebo/issues/237)
1. Ocean currents.

### Fortress

1. [Headless simulation using EGL.](https://github.com/ignitionrobotics/ign-rendering/issues/223)
1. [Upgrade to Ogre 2.2.](https://github.com/ignitionrobotics/ign-rendering/issues/223)
1. [Improve `<pose>` tag on SDFormat.](https://github.com/osrf/sdformat/issues/252)

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
