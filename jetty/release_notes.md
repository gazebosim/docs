{#release-notes}

# Release Notes

## Zenoh in gz-transport

Gazebo now supports Zenoh as an alternative transport implementation, offering
improved discovery, interoperability, and performance. To enable Zenoh, set the
environment variable `export GZ_TRANSPORT_IMPLEMENTATION=zenoh`. This allows
Gazebo to leverage Zenoh's features and potentially integrate more seamlessly
with ROS 2 and other systems utilizing Zenoh.

See the [tracking issue](https://github.com/gazebosim/gz-transport/issues/559)
and the implementation pull request
<https://github.com/gazebosim/gz-transport/pull/665> for more details.

## Remove major version from package names

We have refactored package naming to remove major version numbers, simplifying
dependency management and reducing maintenance overhead. This change impacts how
Gazebo packages are identified in CMake builds, package installations, and
import statements.

See the
[tracking issue](https://github.com/gazebo-tooling/release-tools/issues/1244)
for more details.

## Migrate from Qt5 to Qt6

With Qt5 reaching
[end of life](https://www.qt.io/blog/extended-security-maintenance-for-qt-5.15-begins-may-2025),
Gazebo has been migrated to Qt6. While there are no major changes to end-users
of the Gazebo GUI, `gz-gui` plugin authors will need to make changes to ensure
compatibility with Qt6. Make sure to check out the
[migration guide](https://gazebosim.org/api/gui/10/migration_qt6.html).

See the [tracking issue](https://github.com/gazebosim/gz-gui/issues/586) and the
implementation pull requests <https://github.com/gazebosim/gz-gui/pull/666> and
<https://github.com/gazebosim/gz-sim/pull/2832> for more details

## Implement ROS standard simulation interfaces

Gazebo now supports the standard ROS 2
[simulation interfaces](https://github.com/ros-simulation/simulation_interfaces),
enabling seamless integration with ROS 2 ecosystems and providing access to
features for controlling and querying the simulation state. Documentation has
also been added to guide users on utilizing these new interfaces.

See the [tracking issue](https://github.com/gazebosim/ros_gz/issues/732) and the
implementation pull requests <https://github.com/gazebosim/ros_gz/pull/790> and
<https://github.com/gazebosim/docs/pull/601> for more details

## Resolve auto inertia based on input mass

Automatically computed inertial can now use the `mass` specified in the SDF file
and no longer requires setting the density of collision objects. When both mass
and density are set, Gazebo now correctly scales the auto-computed inertia based
on the specified mass, respecting the density ratios between collisions.

See the [tracking issue](https://github.com/gazebosim/sdformat/issues/1482) and
the implementation pull request
<https://github.com/gazebosim/sdformat/pull/1513> for more details

## Improve Gazebo APIs for Reinforcement Learning

Several APIs have been improved to streamline reinforcement learning pipelines.
We have also added an [example integration with StableBaselines3](https://github.com/gazebosim/gz-sim/tree/gz-sim10/examples/scripts/reinforcement_learning/simple_cart_pole)
for reinforcement learning, enabling users to experiment with RL algorithms within
Gazebo. The example provides a starting point for developing and testing robotic
control policies.

```{figure} https://github.com/user-attachments/assets/f30160a3-e04f-4ec1-aab4-111739b0d349
:width: 400
:alt: RL_with_gazebo_simple_example

Example of doing Reinforcement Learning in Gazebo
```

See the [tracking issue](https://github.com/gazebosim/gz-sim/issues/2662) and
the implementation pull requests <https://github.com/gazebosim/gz-sim/pull/2667>
and <https://github.com/gazebosim/gz-sim/pull/2647> for more details

## Standalone executables for the `gz` tool

Previously, the `gz` tool, a Ruby-based CLI interface, offered subcommands by
loading shared libraries provided by each of the Gazebo libraries. This approach
had several downsides including difficulty with debugging with `gdb`,
portability issues when running on Windows or macOS. With the new approach each
of the libraries provide standalone executables that are spawn by the `gz` tool.
The standalone executables can be executed directly if desired, making it
significantly easier to run under `gdb`. Unlike loading shared libraries,
running executables is much more straightforward and does not have any of the
portability issues of the previous approach.

See the [tracking issue](https://github.com/gazebosim/gz-tools/issues/7) and one
of the implementation pull requests
<https://github.com/gazebosim/gz-sim/pull/2849> for more details

## Parallel asset download

This enhancement significantly improves Gazebo's startup time and responsiveness
when loading worlds with many assets. Assets are now downloaded in parallel,
allowing the GUI to remain interactive during the loading process and enabling
users to close the window if needed.

See the [tracking issue](https://github.com/gazebosim/gz-sim/issues/1260) and
one of the implementation pull requests
<https://github.com/gazebosim/gz-sim/pull/2992> for more details

## Occupancy Grid Export

This enhancement adds a plugin that enables end users to directly export occupancy
grids for use with Nav2 or other mobile robotics software. 

See the pull request <https://github.com/gazebosim/gz-sim/pull/2958> for more details.

