# Gazebo Release Features

Here you will find the set of features that are available in each release.

Take a look at the [Roadmap](/docs/all/roadmap) for information about upcoming
features, some of which may land in released versions of Gazebo.

## Harmonic

For a complete list of features included in the initial release of Harmonic, please see the [highlights](https://github.com/gazebosim/gz-harmonic/blob/main/highlights.md) and [release notes](https://github.com/gazebosim/gz-harmonic/blob/main/release_notes.md) documentation in the Gazebo Harmonic repository.

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       3.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/gz-cmake3/Changelog.md)     |
|   gz-common       |       5.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/gz-common5/Changelog.md)    |
|   gz-fuel-tools   |       9.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/gz-fuel-tools9/Changelog.md)    |
|   gz-gui          |       8.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/gz-gui8/Changelog.md)       |
|   gz-launch       |       7.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/gz-launch7/Changelog.md)
|   gz-math         |       7.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/gz-math7/Changelog.md)
|   gz-msgs         |      10.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/gz-msgs10/Changelog.md)
|   gz-physics      |       7.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/gz-physics7/Changelog.md)
|   gz-plugin       |       2.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/gz-plugin2/Changelog.md)     |
|   gz-rendering    |       8.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/Changelog.md)      |
|   gz-sensors      |       8.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/Changelog.md)      |
|   gz-sim          |       8.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/gz-sim8/Changelog.md)     |
|   gz-tools        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/gz-tools2/Changelog.md)     |
|   gz-transport    |      13.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/gz-transport13/Changelog.md)      |
|   gz-utils        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-utils/blob/gz-utils2/Changelog.md)      |
|   sdformat        |      14.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf14/Changelog.md)        |

## Garden

1. [Point cloud visualization.](https://github.com/gazebosim/gz-sim/issues/1156)
1. [Wide angle camera (Ogre 1).](https://github.com/gazebosim/gz-sensors/issues/24)
1. [DEM heightmaps.](https://github.com/gazebosim/gz-sim/issues/235)
1. [Camera distortion.](https://github.com/gazebosim/gz-sensors/issues/107)
1. [Wind effects configurable on a location basis.](https://github.com/gazebosim/gz-sim/pull/1357)
1. [Simulation Reset API.](https://github.com/gazebosim/gz-sim/issues/1107)
1. [Renaming Ignition to Gazebo.](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356)
1. [glTF and GLB mesh support.](https://github.com/gazebosim/gz-common/issues/344)
1. [Vulkan support (Ogre 2.3)](https://github.com/gazebosim/gz-rendering/pull/553)
1. [Lunar terrain and coordinates.](https://github.com/gazebosim/sdformat/pull/1050)
1. [Static plugins.](https://github.com/gazebosim/gz-plugin/pull/97)
1. [Conversion between SDF and MJCF.](https://github.com/gazebosim/gz-mujoco/tree/main/sdformat_mjcf)
1. [Python API for SDFormat.](http://sdformat.org/tutorials?tut=python_bindings&cat=developers&)
1. [Material shininess.](https://github.com/gazebosim/sdformat/pull/985)
1. [Improve Bullet support (new gz-physics plugin for Bullet's Featherstone API).](https://github.com/gazebosim/gz-physics/issues/44)
1. [Generic physics tests.](https://github.com/gazebosim/gz-physics/issues/50)
1. [Custom rendering sensors.](https://github.com/gazebosim/gz-sim/pull/1804)[^1]
1. [Environmental lookup sensors.](https://github.com/gazebosim/gz-sim/pull/1616)
1. [Allow loading SDFormat model files from the command line (instead of world files only).](https://github.com/gazebosim/gz-sim/pull/1775)
1. [Simple acoustic communication plugin.](https://github.com/gazebosim/gz-sim/pull/1704)
1. [Airspeed sensor.](https://github.com/gazebosim/gz-sensors/pull/305)
1. [Hydrodynamic added mass.](https://github.com/gazebosim/gz-sim/issues/1462)
1. [Magnetometer value based on location.](https://github.com/gazebosim/gz-sim/pull/1907)
1. [Lens flare support for Ogre and Ogre2.](https://github.com/gazebosim/gz-rendering/issues/730)
1. [Allow specifying initial simulation time with a CLI argument](https://github.com/gazebosim/gz-sim/pull/1801)
1. [Joints defined at the world level](https://github.com/gazebosim/sdformat/issues/1115)
1. [Dynamic wrench application through the GUI (Mouse interaction with simulated models)](https://github.com/gazebosim/gz-sim/issues/306)

[^1]: All the functionality for creating custom rendering sensors is in Garden, but the PR listed, which serves as an
  example, is only available in Harmonic.

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       3.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/gz-cmake3/Changelog.md)     |
|   gz-common       |       5.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/gz-common5/Changelog.md)    |
|   gz-fuel-tools   |       8.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/gz-fuel-tools8/Changelog.md)    |
|   gz-gui          |       7.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/gz-gui7/Changelog.md)       |
|   gz-launch       |       6.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/gz-launch6/Changelog.md)
|   gz-math         |       7.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/gz-math7/Changelog.md)
|   gz-msgs         |       9.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/gz-msgs9/Changelog.md)
|   gz-physics      |       6.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/gz-physics6/Changelog.md)
|   gz-plugin       |       2.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/gz-plugin2/Changelog.md)     |
|   gz-rendering    |       7.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/gz-rendering7/Changelog.md)      |
|   gz-sensors      |       7.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/gz-sensors7/Changelog.md)      |
|   gz-sim          |       7.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/gz-sim7/Changelog.md)     |
|   gz-tools        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/gz-tools2/Changelog.md)     |
|   gz-transport    |      12.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/gz-transport12/Changelog.md)      |
|   gz-utils        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-utils/blob/gz-utils2/Changelog.md)      |
|   sdformat        |      13.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf13/Changelog.md)        |

## Fortress

1. [Headless simulation using EGL.](https://github.com/gazebosim/gz-rendering/issues/223)
1. [Refactor ECM::Each for performance.](https://github.com/gazebosim/gz-sim/issues/711)
1. [Upgrade to Ogre 2.2.](https://github.com/gazebosim/gz-rendering/issues/223)
1. [Improve `<pose>` tag on SDFormat.](https://github.com/osrf/sdformat/issues/252)
1. [Heightmaps on Ogre 2](https://github.com/gazebosim/gz-rendering/issues/187)
1. [Spherical coordinates](https://github.com/gazebosim/gz-sim/issues/981)
1. [Buoyancy engine](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/buoyancy_engine.sdf)
1. [Control lights from ROS 2](https://github.com/gazebosim/ros_gz/pull/187)
1. [Python interface to Gazebo.](https://github.com/gazebosim/gz-sim/issues/789)
1. [Custom shaders.](https://github.com/gazebosim/gz-sim/issues/657)
1. [Visual plugins.](https://github.com/gazebosim/gz-sim/issues/265)
1. [Rendering waves.](https://github.com/gazebosim/gz-rendering/pull/541)
1. [Generic comms system.](https://github.com/gazebosim/gz-sim/pull/1416)
1. [Wheel slip commands.](https://github.com/gazebosim/gz-sim/pull/1241)
1. [USD importer / exporter.](https://github.com/gazebosim/sdformat/tree/sdf12/examples/usdConverter)
1. [Bridge Gazebo services to ROS 2 services.](https://github.com/gazebosim/ros_ign/pull/211)
1. [Omniverse application](https://github.com/gazebosim/gz-omni)
1. [Pose and Twist with covariance.](https://github.com/gazebosim/gz-msgs/pull/224)
1. [Kinematic loops for detachable joints.](https://github.com/gazebosim/gz-physics/pull/352)
1. [ROS bridge configurable via YAML.](https://github.com/gazebosim/ros_gz/pull/238)
1. [ROS bridge as a node component.](https://github.com/gazebosim/ros_gz/pull/238)
1. [SDF support on ROS 2.](https://github.com/gazebosim/ros_gz/pull/265)
1. Sensors
    1. [Custom sensors.](https://gazebosim.org/api/sensors/6.0/custom_sensors.html)
    1. [Segmentation camera.](https://gazebosim.org/api/sensors/6.0/segmentationcamera_igngazebo.html)
    1. [Joint force-torque sensor.](https://github.com/gazebosim/gz-sensors/issues/25)
    1. [GPS / NavSat sensor.](https://github.com/gazebosim/gz-sensors/issues/23)
    1. [Triggered cameras.](https://github.com/gazebosim/gz-sensors/issues/185)
    1. [Bounding box camera.](https://github.com/gazebosim/gz-sensors/issues/135)
1. GUI features
    1. [Consolidate Scene3D with GzScene3D](https://github.com/gazebosim/gz-gui/issues/137)
    1. [Visualize wireframes](https://github.com/gazebosim/gz-sim/pull/816)
    1. [Visualize transparent](https://github.com/gazebosim/gz-sim/pull/878)
    1. [Visualize inertia](https://github.com/gazebosim/gz-sim/issues/111)
    1. [Visualize center of mass](https://github.com/gazebosim/gz-sim/issues/110)
    1. [Visualize joints](https://github.com/gazebosim/gz-sim/issues/106)
    1. [Orthographic view](https://github.com/gazebosim/gz-sim/issues/103)
    1. [System inspector.](https://github.com/gazebosim/gz-sim/issues/191)
1. [Hydrodynamics for water surface vehicles.](https://github.com/gazebosim/gz-sim/pull/818)
1. [Custom skybox from SDF.](https://github.com/gazebosim/sdformat/pull/1037)
1. [Gz3D: support heightmaps, skybox and particles.](https://github.com/gazebo-web/gzweb/pull/35)
1. [Tools for creating new Gazebo projects](https://github.com/gazebosim/gz_pkg_create)
1. [Simplify determination of a sensor's topic name for rendering sensors.](https://github.com/gazebosim/gz-sim/pull/1908)
1. [APIs that ease migration from classic.](https://github.com/gazebosim/gz-sim/issues/325)

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       4.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common4/Changelog.md)    |
|   gz-fuel-tools   |       7.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools7/Changelog.md)    |
|   gz-sim          |       6.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/Changelog.md)     |
|   gz-gui          |       6.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/ign-gui6/Changelog.md)       |
|   gz-launch       |       5.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/ign-launch5/Changelog.md)
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)
|   gz-msgs         |       8.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs8/Changelog.md)
|   gz-physics      |       5.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics5/Changelog.md)
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       6.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/ign-rendering6/Changelog.md)      |
|   gz-sensors      |       6.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/Changelog.md)      |
|   gz-tools        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools1/Changelog.md)     |
|   gz-transport    |      11.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport11/Changelog.md)      |
|   gz-utils        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-utils/blob/ign-utils1/Changelog.md)      |
|   sdformat         |      12.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf12/Changelog.md)        |

## Edifice (EOL)

1. New utility library with minimal dependencies: [Gazebo Utils](https://github.com/gazebosim/gz-utils/).
1. [Sky box support.](https://github.com/gazebosim/gz-rendering/issues/98)
1. [Lightmap support.](https://github.com/gazebosim/gz-sim/pull/471)
1. [Capsule and ellipsoid geometries.](https://github.com/osrf/sdformat/issues/376)
1. [SDF model composition.](https://github.com/osrf/sdformat/issues/278)
1. [SDFormat interface for non-SDF models.](http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&#1-5-minimal-libsdformat-interface-types-for-non-sdformat-models)
1. [Choose render order for overlapping polygons.](https://github.com/gazebosim/gz-rendering/pull/188)
1. [Light visualization.](https://github.com/gazebosim/gz-sim/issues/193)
1. [Spawn lights from the GUI.](https://github.com/gazebosim/gz-sim/issues/119)
1. [Mecanum wheel controller.](https://github.com/gazebosim/gz-sim/issues/579)
1. [Hydrodynamics.](https://gazebosim.org/api/gazebo/5.0/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html)
1. [Ocean currents.](https://github.com/gazebosim/gz-sim/pull/800)
1. [Hook command line tool to binaries instead of libraries.](https://github.com/gazebosim/gz-tools/issues/7)
1. [Heightmap support using Ogre 1 and DART.](https://github.com/gazebosim/gz-sim/issues/237)
1. [Turn lights on and off.](https://github.com/gazebosim/gz-sim/pull/1343)
1. [Toggle light visualization.](https://github.com/gazebosim/gz-sim/issues/638)
1. [Model photoshoot plugin.](https://github.com/gazebosim/gz-sim/pull/1331)

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       4.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common4/Changelog.md)    |
|   gz-fuel-tools   |       6.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools6/Changelog.md)    |
|   gz-sim          |       5.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo5/Changelog.md)     |
|   gz-gui          |       5.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/ign-gui5/Changelog.md)       |
|   gz-launch       |       4.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/ign-launch4/Changelog.md)
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)
|   gz-msgs         |       7.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/Changelog.md)
|   gz-physics      |       4.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics4/Changelog.md)
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       5.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/ign-rendering5/Changelog.md)      |
|   gz-sensors      |       5.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors5/Changelog.md)      |
|   gz-tools        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools1/Changelog.md)     |
|   gz-transport    |      10.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport10/Changelog.md)      |
|   gz-utils        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-utils/blob/ign-utils1/Changelog.md)      |
|   sdformat         |      11.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf11/Changelog.md)        |

## Dome (EOL)

1. Particle effects on [Gazebo Rendering](https://gazebosim.org/api/rendering/4.1/particles.html) and [Gazebo Sim](https://github.com/gazebosim/gz-sim/blob/ign-gazebo4/examples/worlds/particle_emitter.sdf).
1. Actor plugins.
1. Efficient skeleton animations.
1. [Optical tactile sensor plugin.](https://community.gazebosim.org/t/gsoc-2020-ignition-gazebo-optical-tactile-sensor-plugin/618)
1. [Support entity names with spaces.](https://github.com/gazebosim/gz-sim/issues/239)
1. Kinetic energy monitor plugin.
1. [Texture-based thermal signature](https://gazebosim.org/api/sensors/4.1/thermalcameraigngazebo.html) for objects, visible from thermal camera.
1. [Web visualization of running simulations](https://gazebosim.org/docs/dome/web_visualization).
1. [Bullet physics engine.](https://github.com/gazebosim/gz-physics/issues/44)
1. [Parametrized SDF files.](http://sdformat.org/tutorials?tut=param_passing_proposal)
1. [libSDFormat now uses gz-cmake](https://github.com/gazebosim/sdformat/issues/181)
1. GUI tools:
    * [Plotting](https://community.gazebosim.org/t/gsoc-2020-plotting-tool-for-ignition/619)
    * [Lidar visualization](https://community.gazebosim.org/t/gsoc-2020-sensor-data-visualization/638)
    * [Configure physics real time factor and step size.](https://github.com/gazebosim/gz-sim/pull/536)
    * [Configure lights from the GUI or transport.](https://github.com/gazebosim/gz-sim/issues/122)
    * [Contact visualization.](https://github.com/gazebosim/gz-sim/issues/234)

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       3.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common3/Changelog.md)    |
|   gz-fuel-tools   |       5.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools5/Changelog.md)    |
|   gz-sim          |       4.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo4/Changelog.md)     |
|   gz-gui          |       4.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/ign-gui4/Changelog.md)       |
|   gz-launch       |       3.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/ign-launch3/Changelog.md)
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)
|   gz-msgs         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs6/Changelog.md)
|   gz-physics      |       3.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics3/Changelog.md)
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       4.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/ign-rendering4/Changelog.md)      |
|   gz-sensors      |       4.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors4/Changelog.md)      |
|   gz-tools        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools1/Changelog.md)     |
|   gz-transport    |       9.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport9/Changelog.md)      |
|   sdformat         |      10.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf10/Changelog.md)        |

## Citadel (LTS)

1. Visual markers.
1. Animated actors for Ogre 1 and Ogre 2.
1. Thermal camera sensor.
1. Emissive texture maps.
1. SDFormat frame semantics.
1. Upload and delete models to Fuel from command line.
1. Buoyancy model.
1. [Trivial Physics Engine](https://community.gazebosim.org/t/announcing-new-physics-engine-tpe-trivial-physics-engine/629)
1. [Tutorial series](https://community.gazebosim.org/t/gsoc-2020-new-ignition-gazebo-demos/613).
1. [Improved Gazebo Physics documentation.](https://community.gazebosim.org/t/gsod-2020-ignition-physics-tutorial-and-api-documentation/746)
1. [Custom rendering engines.](https://gazebosim.org/api/rendering/3.2/renderingplugin.html)
1. [Logical audio sensor and source.](https://gazebosim.org/api/gazebo/3.7/logicalaudiosensor.html)
1. Lockstepped video recording that can optionally use simulation time instead of real time.
1. [Easier migration of SDF files from Gazebo classic.](https://gazebosim.org/api/gazebo/3.7/migrationsdf.html)
1. [Binary Windows install through conda-forge.](https://gazebosim.org/docs/citadel/install_windows)
1. [Restitution coefficient (bouncing).](https://github.com/gazebosim/gz-physics/pull/139)
1. [Custom retroreflection for objects detected by lidar.](https://github.com/gazebosim/gz-sim/blob/ign-gazebo3/examples/worlds/gpu_lidar_retro_values_sensor.sdf)
1. [More SDFormat documentation](http://sdformat.org/tutorials).
1. Load PBR materials from OBJ.
1. [Model info CLI.](https://github.com/gazebosim/gz-sim/issues/313)
1. [Test fixture to easily run simulation-based automated tests.](https://gazebosim.org/api/gazebo/3.9/test_fixture.html)
1. [Drag and drop meshes into the scene.](https://github.com/gazebosim/gz-sim/pull/939)
1. [Tracked vehicles](https://github.com/gazebosim/gz-sim/pull/869)
1. [Python interface for Gazebo Math.](https://github.com/gazebosim/gz-math/issues/210)
1. [Integration with ros2_control.](https://github.com/gazebosim/gz_ros2_control/)
1. [gz_ros2_control support for various models](https://github.com/gazbosim/docs/issues/222)
1. [Polyline extruded 2D geometries](https://github.com/gazebosim/docs/issues/186)
1. [Apply forces and torques via transport or SDF](https://github.com/gazebosim/gz-sim/pull/1593)
1. New graphical interfaces:
    * Widget listing all transport topics.
    * Widget that publishes keys pressed on the keyboard.
    * Collision visualization.
    * [Screenshot widget.](https://gazebosim.org/api/gui/3.5/screenshot.html)
    * [Joint position controller.](https://app.gazebosim.org/OpenRobotics/fuel/worlds/NAO%20joint%20control)
    * [2D teleop widget.](https://github.com/gazebosim/gz-gui/issues/186)
    * [3D plot.](https://github.com/gazebosim/gz-sim/issues/231)
    * [Notification snackbar](https://github.com/gazebosim/gz-gui/issues/44)
   1. [Quick start dialog.](https://github.com/gazebosim/gz-sim/issues/1252)
1. APIs that make it easier to migrate from Gazebo classic:
    * [World](https://gazebosim.org/api/gazebo/3.5/migrationworldapi.html)
    * [Model](https://gazebosim.org/api/gazebo/3.5/migrationmodelapi.html)
    * [Link](https://gazebosim.org/api/gazebo/3.5/migrationlinkapi.html)
1. [Expose camera calibration parameters.](https://github.com/gazebosim/sdformat/pull/1088)
1. [Speed up Resource Spawner.](https://github.com/gazebosim/gz-sim/issues/1936)
1. [Bash completion](https://github.com/gazebosim/gz-tools/issues/1)

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       3.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common3/Changelog.md)    |
|   gz-fuel-tools   |       4.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools4/Changelog.md)    |
|   gz-sim          |       3.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo3/Changelog.md)     |
|   gz-gui          |       3.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/ign-gui3/Changelog.md)       |
|   gz-launch       |       2.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/ign-launch2/Changelog.md)
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)
|   gz-msgs         |       5.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs5/Changelog.md)
|   gz-physics      |       2.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics2/Changelog.md)
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       3.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/ign-rendering3/Changelog.md)      |
|   gz-sensors      |       3.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors3/Changelog.md)      |
|   gz-tools        |       1.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools1/Changelog.md)     |
|   gz-transport    |       8.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport8/Changelog.md)      |
|   sdformat         |       9.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf9/Changelog.md)        |

## Blueprint (EOL)

1. [Physically based rendering (PBR)](https://en.wikipedia.org/wiki/Physically_based_rendering) materials.
1. Air pressure, RGBD and stereo camera sensors.
1. Global wind model.
1. Joint state publisher.
1. Support for UAVs.
1. Integration of `ign` command line tool into Gazebo Sim.
1. Logging and playback of simulation state.
1. Command line tools to control log playback.
1. Battery model based on vehicle motion and rechargeable batteries.
1. Integration of [Google benchmark](https://github.com/google/benchmark) for performance metrics and analysis.
1. Tracked vehicle support.
1. Breadcrumbs plugin.
1. Position-based PID controller.
1. Improved resource path handling.
1. Loading custom physics engine plugins.
1. Plugin that publishes a user specified message on an output topic in response to an input message.
1. Noise for RGBD camera.
1. Load worlds from Fuel.
1. [Customizable GUI layout](https://gazebosim.org/api/gazebo/3.3/gui_config.html).
1. [Detachable joints](https://gazebosim.org/api/gazebo/4.0/detachablejoints.html)
1. GUI tools:
    * GUI tools for model placement, and a new Scene Tree widget.
    * Translate and rotate models.
    * Entity tree.
    * Video recorder.
    * Move to models.
    * Follow model.
    * Delete model.
    * Grid.
    * Drag-and-drop models from Fuel to Gazebo Sim UI.
    * Preset view angles.
    * Hotkeys for transform modes and snapping.
    * Entity selection.
    * [Align models](https://gazebosim.org/docs/dome/manipulating_models#align-tool).
    * Insert simple shapes.
    * Insert models from online sources and local directories.
    * Log playback scrubber.
    * Save worlds.
    * Tape measure.

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       3.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common3/Changelog.md)    |
|   gz-fuel-tools   |       3.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools3/Changelog.md)    |
|   gz-sim          |       2.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo2/Changelog.md)     |
|   gz-gui          |       2.x     |       [Changelog](https://github.com/gazebosim/gz-gui/blob/ign-gui2/Changelog.md)       |
|   gz-launch       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-launch/blob/ign-launch1/Changelog.md)
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)
|   gz-msgs         |       4.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs4/Changelog.md)
|   gz-physics      |       1.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics1/Changelog.md)
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       2.x     |       [Changelog](https://github.com/gazebosim/gz-rendering/blob/ign-rendering2/Changelog.md)      |
|   gz-sensors      |       2.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors2/Changelog.md)      |
|   gz-tools        |       0.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools0/Changelog.md)
|   gz-transport    |       7.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport7/Changelog.md)
|   sdformat         |       8.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf8/Changelog.md)        |

## Acropolis (EOL)

The first major release of Gazebo focused on the basics of simulation. The basics primarily encompassed integration of physics, sensors, graphical tools, and programmatic interfaces.

1. Support for [DART](https://dartsim.github.io/) in [Gazebo Physics](/libs/physics).
2. Ogre1.9 and Ogre2.1 support in [Gazebo Rendering](/libs/rendering)
3. [Entity Component System](https://en.wikipedia.org/wiki/Entity_component_system) based simulation engine in [Gazebo Sim](/libs/gazebo).
4. A sensor suite that includes contact sensor, logical camera, monocular camera, depth camera, LIDAR, magnetometer, altimeter, and IMU is available through [Gazebo Sensors](/libs/sensors) and [Gazebo Sim](/libs/gazebo).
5. [Launch system](/libs/launch) capable of running and managing a set of plugins and executables.
6. Cloud-hosted simulation assets provided by [app.gazebosim.org](https://app.gazebosim.org).
7. Distributed simulation using lock-stepping.
8. Dynamic loading/unloading of simulation models based on the location of performer, usually a robot.
9. Simulation state logging.
10. Plugin-based GUI system based on [QtQuick](https://en.wikipedia.org/wiki/Qt_Quick) and [Material Design](https://material.io/design/). Available
    plugins include 3D scene viewer, image viewer, topic echo, topic
    publisher, world control, and world statistics.

The Acropolis collection is composed by many different Gazebo libraries. The
collection assures that all libraries all compatible and can be used together.

| Library name       | Version       | Changelog     |
| ------------------ |:-------------:|:-------------:|
|   gz-cmake        |       2.x     |       [Changelog](https://github.com/gazebosim/gz-cmake/blob/ign-cmake2/Changelog.md)     |
|   gz-common       |       3.x     |       [Changelog](https://github.com/gazebosim/gz-common/blob/ign-common3/Changelog.md)    |
|   gz-fuel-tools   |       3.x     |       [Changelog](https://github.com/gazebosim/gz-fuel-tools/blob/ign-fuel-tools3/Changelog.md)    |
|   gz-sim          |       1.x     |       [Changelog](https://github.com/gazebosim/gz-sim/blob/ign-gazebo1/Changelog.md)     |
|   gz-gui          |       1.x     |       None       |
|   gz-launch       |       0.x     |       None       |
|   gz-math         |       6.x     |       [Changelog](https://github.com/gazebosim/gz-math/blob/ign-math6/Changelog.md)     |
|   gz-msgs         |       3.x     |       [Changelog](https://github.com/gazebosim/gz-msgs/blob/ign-msgs3/Changelog.md)     |
|   gz-physics      |       1.x     |       [Changelog](https://github.com/gazebosim/gz-physics/blob/ign-physics1/Changelog.md)      |
|   gz-plugin       |       1.x     |       [Changelog](https://github.com/gazebosim/gz-plugin/blob/ign-plugin1/Changelog.md)     |
|   gz-rendering    |       1.x     |       None      |
|   gz-sensors      |       1.x     |       [Changelog](https://github.com/gazebosim/gz-sensors/blob/ign-sensors1/Changelog.md)      |
|   gz-tools        |       0.x     |       [Changelog](https://github.com/gazebosim/gz-tools/blob/ign-tools0/Changelog.md)     |
|   gz-transport    |       6.x     |       [Changelog](https://github.com/gazebosim/gz-transport/blob/ign-transport6/Changelog.md)      |
|   sdformat         |       8.x     |       [Changelog](https://github.com/osrf/sdformat/blob/sdf8/Changelog.md)        |
