# Gazebo Classic Migration

Gazebo was started in 2002. After over 15 years of development it was time for a
significant upgrade and modernization. This upgrade also provided the
opportunity to move away from a monolithic architecture to a collection of
loosely coupled libraries.

These collection of libraries make up the new Gazebo. As a convention we refer
to older versions of Gazebo, those with release numbers like Gazebo 9 and Gazebo
11 as "Gazebo Classic". Newer versions of Gazebo, formerly called "Ignition",
with lettered releases names like Harmonic, are referred to as just "Gazebo".

:::{tip}

Since the name of the project has gone through two major changes, we highly
recommend you read the [history](https://gazebosim.org/about) of the project as
well as our
[community post](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356) to
have a better understanding of the terminology used on this website.

:::

Here you'll find guides and resources for migrating existing Gazebo Classic
projects to the new Gazebo.

- [Migrating ROS 2 packages that use Gazebo Classic](migrating_gazebo_classic_ros2_packages)
- [Installing Gazebo11 side by side with new Gazebo](install_gz11_side_by_side)
- Migration from Gazebo Classic: Plugins -
  [Fortress](https://gazebosim.org/api/gazebo/6/migrationplugins.html) |
  [Harmonic](https://gazebosim.org/api/sim/8/migrationplugins.html)
- Migration from Gazebo classic: SDF -
  [Fortress](https://gazebosim.org/api/gazebo/6/migrationsdf.html) |
  [Harmonic](https://gazebosim.org/api/sim/8/migrationsdf.html)
- Case study: migrating the ArduPilot ModelPlugin from Gazebo classic to
  Gazebo - [Fortress](https://gazebosim.org/api/gazebo/6/ardupilot.html) |
  [Harmonic](https://gazebosim.org/api/sim/8/ardupilot.html)
- [Basic description of SDF worlds](sdf_worlds)
- [Feature Comparison with Gazebo Classic](comparison)
- [Documentation for ros_gz](ros2_integration)
- List of Systems (plugins):
  [Fortress](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo_1_1systems.html)
  |
  [Harmonic](https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html)
