# Ignition Robotics Roadmap

This page describes planned work for Ignition Robotics. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition Robotics. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Quarterly Roadmap

**Close the Gap** is Ignition's current focus topic. Over the next two
quarters we will concentrate development efforts on topics that reduce
feature disparity between Gazebo and Ignition and facilitate migration from
Gazebo to Ignition. Take a look at the
[feature comparison](/docs/citadel/comparison) page for a list of
differences between Gazebo and Ignition.

## 2020 Q3(Jul - Sep)

* **Migration Strategies**: Develop and document strategies for migrating
plugins, SDF files, and other simulation resources from Gazebo to Ignition.
    * See [all open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22migration%22&state=open&type=Issues)
      with the "migration" label.
    * See [current development status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22migration%22)
      for tickets with the "migration" label.

* **Documentation**: Improve documentation release process and usability.
Port relevant Gazebo tutorials to Ignition.
    * See [all open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22boost+the+docs%22&state=open&type=Issues)
      with the "boost the docs" label.
    * See [current development status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22boost+the+docs%22)
      for tickets with the "boost the docs" label.

## 2020 Q4(Oct - Dec)

* **Cross platform compatibility**: Fully support Windows, Ubuntu, and macOS.
    * Windows
        * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3AWindows&type=Issues)
        * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3AWindows)
    * macOS
        * [All open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3AmacOS&type=Issues)
        * [Status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3AmacOS)

* **Feature parity**: Port features from Gazebo that are missing in Ignition.
    * See [all open tickets](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22close+the+gap%22&state=open&type=Issues)
      with the "close the gap" label.
    * See [current development status](https://github.com/orgs/ignitionrobotics/projects/3?card_filter_query=label%3A%22close+the+gap%22)
      for tickets with the "close the gap" label.

## Feature Roadmap

A number of features are planned for specific releases of Ignition. The
features listed here derive from the Quarterly Roadmap, listed above, and other
ongoing projects.  See the [Release Features](/docs/release-features) page for a list of features already available in each release of Ignition.

A named release of Ignition, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Each library follows
[semantic versioning](https://semver.org/), and we make use of the [PImpl
idom](https://en.cppreference.com/w/cpp/language/pimpl). This development
and release pattern allows us to distribute patch and minor updates into an already released Ignition version. For example, if a new feature does not break API/ABI then we will target the feature to the oldest compatible non-EOL release and propagate the feature forward.

### Blueprint

1. Detachable joints.
1. Buoyancy model.
1. Additional graphical tools for model and world creation and editing.
1. GUI tool to insert models from online sources and local directories.
1. Audio sensor and source.
1. Improved resource path handling.
1. Loading custom physics engine plugins.

### Citadel

1. Simplified physics engine.
1. Bazel build files.
1. SDF website update, documentation, and model composition.
1. Trajectory animation.

### Dome

1. Particle effects, to support smoke and gas.
1. Efficient skeleton animations.
1. Localized wind (wind that is constrained to a region of influence).
1. Design for Enhanced distributed simulation.
1. OpenGL shaders to simulate underwater effects in camera sensors.
1. Mesh level of detail support.

## Planned releases

Please see the [Releases](/docs/all/releases) for the timeline of and information about future distributions.

## Contributing to Ignition

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](/docs/all/contributing) guide.
1. [Feature comparision](/docs/citadel/comparison) list. This page lists the
   feature gaps between Gazebo-classic and Ignition Gazebo.
1. Take a look at the various [libraries](/libs), and the issue tracker
   associated with each.
