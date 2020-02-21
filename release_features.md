# Ignition Robotics Release Features

Here you will find the set of features that are available in each release.

Take a look at the [Roadmap](/docs/roadmap) for information about upcoming
features, some of which may land in released versions of Ignition.

## Acropolis

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

## Blueprint

1. [Physically based rendering (PBR)](https://en.wikipedia.org/wiki/Physically_based_rendering) materials.
1. GUI tools for model placement, and a new Scene Tree widget.
1. Air pressure, RGBD and stereo camera sensors.
1. Global wind model.
1. Joint state publisher.
1. Support for UAV vehicles.
1. Integration of `ign` command line tool into Ignition Gazebo.
1. Logging and playback of simulation state.
1. Command line tools to control log playback.
1. Battery model based on vehicle motion.
1. Integration of [Google benchmark](https://github.com/google/benchmark) for performance metrics and analysis.
1. Tracked vehicle support.
1. Rechargeable batteries.
1. Breadcrumbs plugin.
1. GUI tools:
    * Translate and rotate models.
    * Entity tree.
    * Video recorder.
    * Move to models.
    * Follow model.
    * Delete model.
    * Grid.

## Citadel (LTS)

1. Visual markers.
1. Animated actors for Ogre 1.
1. Thermal camera sensor.
1. Emissive texture maps.
1. SDFormat frame semantics.
1. Upload models to Fuel from command line.


