# Ignition Robotics Roadmap

This page describes planned work on Ignition Robotics. The set of planned features and development efforts are kept vague, but should provide insight into the overall direction of Ignition Robotics. Currently, and until around 2021, development of Ignition Robotics is closely tied to the [DARPA Subterranean (SubT) Challenge](https://subtchallenge.com). Therefore, you will see features on the roadmap designed to support large environments in which dozens of ground and aerial vehicles can operate in real-time.


## Planned releases

See the [Releases](/docs/releases) page.

## Release Features

Features planned for a particular release may appear in an
earlier release depending on schedule and adherence to semantic versioning.
For example, if a new feature does not break API/ABI then we will strive to
target the feature to the oldest compatible non-EOL release and propagate the feature forward.

### Acropolis

The first major release of Ignition Robotics focused on the basics of simulation. The basics primarily encompassed integration of physics, sensors, graphical tools, and programmatic interfaces.

1. Support for [DART](https://dartsim.github.io/) in [Ignition Physics](/libs/physics).
2. Ogre1.9 and Ogre2.1 support in [Ignition Rendering](/libs/rendering)
3. [Entity Component System](https://en.wikipedia.org/wiki/Entity_component_system) based simulation engine in [Ignition Gazebo](/libs/gazebo).
4. A sensor suite that includes contact sensor, logical camera, monocular camera, depth camera, LIDAR, magnetometer, altimeter, and IMU is available through [Ignition Sensors](/libs/sensors) and [Ignition Gazebo](/libs/gazebo).
5. [Launch system](/libs/launch) capable of running and managing a set of plugins and executables.
6. Cloud-hosted simulation assets provided by [app.ignitionrobotics.org](https://app.ignitionrobotics.org).
7. Distributed simulation using lock-stepping.
8. Dynamic loading/unloading of simulation models based on the location of performer, usually a robot.
9. Simulation state logging.
10. Plugin-based GUI system based on [QtQuick](https://en.wikipedia.org/wiki/Qt_Quick) and [Material Design](https://material.io/design/). Available
    plugins include 3D scene viewer, image viewer, topic echo, topic
    publisher, world control, and world statistics.

### Blueprint

1. [Physically based rendering (PBR)](https://en.wikipedia.org/wiki/Physically_based_rendering) materials.
2. GUI tools for model placement, and a new Scene Tree widget.
3. Air pressure, RGBD and stereo camera sensors.
4. Global wind model.
5. Joint state publisher.
6. Support for UAV vehicles.
7. Integration of `ign` command line tool into Ignition Gazebo.
8. Logging and playback of simulation state.
9. Battery model based on vehicle motion.
10. Integration of [Google benchmark](https://github.com/google/benchmark) for performance metrics and analysis.
11. GUI tools:
    * Translate and rotate models.
    * Entity tree.
    * Video recorder.
    * Move to models.

### Citadel

1. Oxygen/gas sensor.
1. Command line tools to control log playback.
1. Visual markers.
1. Detachable joints.
1. Localized wind (wind that is constrained to a region of influence).
1. GUI tool for following models.
1. Animated actors.

## Planned Features

1. Particle effects for smoke and gas.
1. Enhanced distributed simulation, i.e. distributed simulation that is not constrained to lock-stepping.
1. Sonar, force-torque, wide-angle camera sensors.
1. GUI tools for visualization of collision entities and inertia.
1. Additional graphical tools for model and world creation and editing.
1. GUI tool to insert models from online sources and local directories.
1. Cloud-hosted simulation.
1. Battery model based on vehicle payload.
1. Support for the Bullet physics engine.
1. Tracked vehicle support.
1. Thermal camera sensor.
1. Rechargeable batteries.
1. SDFormat frame semantics.
