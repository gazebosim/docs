# Ignition Roadmap

This page describes planned work for Ignition. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

**Close the Gap** is Ignition's current focus topic. Over the next two
quarters we will concentrate development efforts on topics that reduce
feature disparity between Gazebo classic and Ignition and facilitate migration from
Gazebo classic to Ignition. Take a look at the
[feature comparison](/docs/citadel/comparison) page for a list of
differences between classic and Ignition.

## 2020 Q3(Jul - Sep) - finished

* **Migration Strategies**: Develop and document strategies for migrating
plugins, SDF files, and other simulation resources from Gazebo classic to Ignition.

* **Documentation**: Improve documentation release process and usability.
Port relevant Gazebo classic tutorials to Ignition.

See the [Igniton Dome release announcement](https://www.openrobotics.org/blog/2019/12/11/ignition-dome-released)
for some of these features released.

## 2020 Q4(Oct - Dec) - ongoing

* **Cross platform compatibility**: Fully support Windows, Ubuntu, and macOS.
    * Windows
        * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3AWindows&type=Issues)
        * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3AWindows)
    * macOS
        * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3AmacOS&type=Issues)
        * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3AmacOS)

* **Feature parity**: Port features from Gazebo classic that are missing in Ignition.
    * See [all open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22close+the+gap%22&state=open&type=Issues)
      with the "close the gap" label.
    * See [current development status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22close+the+gap%22)
      for tickets with the "close the gap" label.

## Feature Roadmap

A number of features are planned for specific releases of Ignition. The
features listed here derive from the Quarterly Roadmap, listed above, and other
ongoing projects.  See the [Release Features](/docs/all/release-features) page for a list of features already available in each release of Ignition.

A named release of Ignition, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Each library follows
[semantic versioning](https://semver.org/), and we make use of the [PImpl
idom](https://en.cppreference.com/w/cpp/language/pimpl). This development
and release pattern allows us to distribute patch and minor updates into an already released Ignition version. For example, if a new feature does not break API/ABI then we will target the feature to the oldest compatible non-EOL release and propagate the feature forward.

### Blueprint

1. Additional graphical tools for model and world creation and editing.
1. Logical audio sensor and source.

### Citadel

1. SDF website update, more documentation.
1. Ease migration of SDF files from Gazebo classic.
1. [Reset world.](https://github.com/ignitionrobotics/ign-gazebo/issues/203)

### Dome

1. Particle effects, to support smoke and gas.
1. Localized wind (wind that is constrained to a region of influence).
1. Optical tactile plugin.
1. [Improved Ignition Physics documentation.](https://community.gazebosim.org/t/gsod-2020-ignition-physics-tutorial-and-api-documentation/746)
1. [Bazel build files.](https://github.com/ignitionrobotics/ign-bazel)

### Edifice

1. Improved Mac and Windows support
1. Mesh level of detail support.
1. Design for Enhanced distributed simulation.
1. [Heightmap support using Ogre 1 and DART.](https://github.com/ignitionrobotics/ign-gazebo/issues/237)
1. [Sky box support.](https://github.com/ignitionrobotics/ign-rendering/issues/98)
1. [Capsule geometry type in SDFormat.](https://github.com/osrf/sdformat/issues/376)
1. [SDF model composition.](https://github.com/osrf/sdformat/issues/278)

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
