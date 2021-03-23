# Ignition Roadmap

This page describes planned work for Ignition. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

## 2021 Q1(Jan - Mar) - ongoing

* **Windows support**
    * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3AWindows&type=Issues)
    * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3AWindows)
* **Demos**: Create various demos highlighting Ignition's features.
* **Web client**: Visualize running simulations on a web browser.

## 2021 Q2(Apr - Jun) - planned

* **Performance**: Improve Ignition Gazebo's runtime performance
    * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22performance%22&state=open&type=Issues)
    * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22performance%22)

## Feature Roadmap

A number of features are planned for specific releases of Ignition. The
features listed here derive from the Quarterly Roadmap, listed above, and other
ongoing projects.  See the [Release Features](/docs/all/release-features) page for a list of features already available in each release of Ignition.

A named release of Ignition, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Each library follows
[semantic versioning](https://semver.org/), and we make use of the [PImpl
idom](https://en.cppreference.com/w/cpp/language/pimpl). This development
and release pattern allows us to distribute patch and minor updates into an already released Ignition version. For example, if a new feature does not break API/ABI then we will target the feature to the oldest compatible non-EOL release and propagate the feature forward.

### Citadel

1. SDF website update, more documentation.
1. Web client.
1. [Joint controller GUI.](https://github.com/ignitionrobotics/ign-gazebo/issues/192)
1. [GPS sensor.](https://github.com/ignitionrobotics/ign-sensors/issues/23)

### Dome

1. Localized wind (wind that is constrained to a region of influence).
1. [Bazel build files.](https://github.com/ignitionrobotics/ign-bazel)
1. [Bullet physics engine.](https://github.com/ignitionrobotics/ign-physics/issues/44)

### Edifice

1. Improved Mac and Windows support.
1. Mesh level of detail support.
1. Design for Enhanced distributed simulation.
1. [Mecanum wheel demo.](https://github.com/ignitionrobotics/ign-gazebo/issues/579)

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
