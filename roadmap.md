# Gazebo Roadmap

This page describes planned work for Gazebo. The set of planned
features and development efforts should provide insight into the overall
direction of Gazebo. If you would like to
see other features on the roadmap, then please get in touch with us at
info@openrobotics.org.

## Gazebo Jetty Roadmap

* **Improve documentation website (gazebosim.org/docs)**
  * [Make doxygen tutorials from each library accessible on the main tutorials page on gazebosim.org/docs](https://github.com/gazebosim/docs/issues/55)
  * Copy/adapt tutorials from <https://classic.gazebosim.org/tutorials>
* [Migrate from Qt5 to Qt6](https://github.com/gazebosim/gz-gui/issues/586)
* [Use Zenoh in gz-transport](https://github.com/gazebosim/gz-sim/issues/1995)
* [Create or improve interfaces for doing Reinforcement Learning using Gazebo](https://github.com/gazebosim/gz-sim/issues/2662)
* [Create a federated third party plugin ecosystem](https://github.com/gazebosim/gz-transport/issues/559)
* [Command line: use binaries instead of shared libraries](https://github.com/gazebosim/gz-tools/issues/7)
* [Download Fuel models on the background](https://github.com/gazebosim/gz-sim/issues/1260)
* [Support Bazel for all Gazebo libraries](https://github.com/gazebosim/gz-bazel)

Note that some of these roadmap items may be backported to older versions of Gazebo
on a best-effort basis if they do not break API and ABI.
## Planned releases

Please see the [Releases](https://github.com/gazebosim/docs/blob/master/releases.md) for the timeline of and information about future distributions.

## Contributing to Gazebo

Looking for something to work on, or just want to help out? Here are a few
resources to get you going.

1. [How to contribute](contributing) guide.
1. [Feature comparision](/docs/citadel/comparison){.external} list. This page lists the
   feature gaps between Gazebo classic and Gazebo Sim.
1. Take a look at the various [libraries](/libs){.external}, and the issue tracker
   associated with each.
