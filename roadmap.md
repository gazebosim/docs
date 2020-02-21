# Ignition Robotics Roadmap

This page describes planned work for Ignition Robotics. The set of planned
features and development efforts should provide insight into the overall
direction of Ignition Robotics. Development of Ignition Robotics is closely
tied to on-going work with a variety of institutions. If you would like to
see other features on the roadmap, then please get in touch with us via
info@openrobotics.org. 

## Planned releases

See the [Releases](/docs/releases) page.

## Release Features

See the [Release Features](/docs/release-features) page for a list of features
available in each release of Ignition.

A named release of Ignition, such as Acropolis or Blueprint, is tied to
a set of [library](/libs) major versions. Each library follows
[semantic versioning](https://semver.org/), and we make use of the [PImpl
idom](https://en.cppreference.com/w/cpp/language/pimpl). This development
and release pattern allows us to distribute patch and minor updates into an already released Ignition version. For example, if a new feature does not break API/ABI then we will target the feature to the oldest compatible non-EOL release and propagate the feature forward.

1. **Blueprint**
  1. Detachable joints.
  1. Buoyancy model.
  1. Additional graphical tools for model and world creation and editing.
  1. GUI tool to insert models from online sources and local directories.
  1. Audio sensor and source.
1. **Citadel**
  1. Simplified physics engine.
  1. Bazel build files.
  1. SDF website update, documentation, and model composition.
  1. Improved resource path handling.
  1. Trajectory animation.
1. **Dome:**
  1. Particle effects, to support smoke and gas.
  1. Efficient skeleton animations.
  1. Localized wind (wind that is constrained to a region of influence).
  1. Design for Enhanced distributed simulation.
  1. OpenGL shaders to simulate underwater effects in camera sensors. 
  1. Mesh level of detail support.

## Contributing to Ignition

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](/docs/all/contributing) guide.
1. [Feature comparision](/docs/citadel/comparison) list. This page lists the
   feature gaps between Gazebo-classic and Ignition Gazebo.
1. Take a look at the various [libraries](/libs), and the issue tracker
   associated with each.
