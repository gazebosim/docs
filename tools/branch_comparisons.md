# Branch comparisons

These links take to comparisons between branches.

If a cell in the table is `-`, this means that the two Ignition releases use
the same version of this Ignition library. So, for example, `ign-cmake` has a
`-` for B ➡️  C because Blueprint and Citadel use `ign-cmake2`.

When it says "There isn’t anything to compare," this means no pull requests
need to be forward-ported.

Otherwise, those branches could use a forward-port pull request, see
instructions on the
[contributing guide](https://ignitionrobotics.org/docs/all/contributing#process).

* **B**: Blueprint
* **C**: Citadel
* **D**: Dome
* **main**: main branch

> SDFormat is a special case where the release branches should be compared
  against `merge_N_M_reference` branches instead of other release branches.
  Be sure to update those branches before using for comparison.

Library | B ➡️  C | C ➡️  D | D ➡️  main
------- | ------ | ------ | -------
[ign-cmake][ign-cmake] | - | - | [D ➡️  main][ign-cmake-main]
[ign-common][ign-common] | - | - | [D ➡️  main][ign-common-main]
[ign-fuel-tools][ign-fuel-tools] | [3 ➡️  4][ign-fuel-tools-3-4] | [4 ➡️  5][ign-fuel-tools-4-5] | [D ➡️  main][ign-fuel-tools-main]
[ign-gazebo][ign-gazebo] | [2 ➡️  3][ign-gazebo-2-3] | [3 ➡️  4][ign-gazebo-3-4] | [D ➡️  main][ign-gazebo-main]
[ign-gui][ign-gui] | [2 ➡️  3][ign-gui-2-3] | [3 ➡️  4][ign-gui-3-4] | [D ➡️  main][ign-gui-main]
[ign-launch][ign-launch] | [1 ➡️  2][ign-launch-1-2] | [2 ➡️  3][ign-launch-2-3] | [D ➡️  main][ign-launch-main]
[ign-math][ign-math] | - | - | [D ➡️  main][ign-math-main]
[ign-msgs][ign-msgs] | [4 ➡️  5][ign-msgs-4-5] | [5 ➡️  6][ign-msgs-5-6] | [D ➡️  main][ign-msgs-main]
[ign-physics][ign-physics] | [1 ➡️  2][ign-physics-1-2] | [2 ➡️  3][ign-physics-2-3] | [D ➡️  main][ign-physics-main]
[ign-plugin][ign-plugin] | - | - | [D ➡️  main][ign-plugin-main]
[ign-rendering][ign-rendering] | [2 ➡️  3][ign-rendering-2-3] | [3 ➡️  4][ign-rendering-3-4] | [D ➡️  main][ign-rendering-main]
[ign-sensors][ign-sensors] | [2 ➡️  3][ign-sensors-2-3] | [3 ➡️  4][ign-sensors-3-4] | [D ➡️  main][ign-sensors-main]
[ign-tools][ign-tools] | - | - | [D ➡️  main][ign-tools-main]
[ign-transport][ign-transport] | [7 ➡️  8][ign-transport-7-8] | [8 ➡️  9][ign-transport-8-9] | [D ➡️  main][ign-transport-main]
[sdformat][sdformat] | [8 ➡️  9][sdformat-8-9] | [9 ➡️  10][sdformat-9-10] | [10 ➡️  main][sdformat-main]

[ign-cmake]: https://github.com/ignitionrobotics/ign-cmake
[ign-cmake-main]: https://github.com/ignitionrobotics/ign-cmake/compare/main...ign-cmake2

[ign-common]: https://github.com/ignitionrobotics/ign-common
[ign-common-main]: https://github.com/ignitionrobotics/ign-common/compare/main...ign-common3

[ign-fuel-tools]: https://github.com/ignitionrobotics/ign-fuel-tools
[ign-fuel-tools-3-4]: https://github.com/ignitionrobotics/ign-fuel-tools/compare/ign-fuel-tools4...ign-fuel-tools3
[ign-fuel-tools-4-5]: https://github.com/ignitionrobotics/ign-fuel-tools/compare/ign-fuel-tools5...ign-fuel-tools4
[ign-fuel-tools-main]: https://github.com/ignitionrobotics/ign-fuel-tools/compare/main...ign-fuel-tools5

[ign-gazebo]: https://github.com/ignitionrobotics/ign-gazebo
[ign-gazebo-2-3]: https://github.com/ignitionrobotics/ign-gazebo/compare/ign-gazebo3...ign-gazebo2
[ign-gazebo-3-4]: https://github.com/ignitionrobotics/ign-gazebo/compare/ign-gazebo4...ign-gazebo3
[ign-gazebo-main]: https://github.com/ignitionrobotics/ign-gazebo/compare/main...ign-gazebo4

[ign-gui]: https://github.com/ignitionrobotics/ign-gui
[ign-gui-2-3]: https://github.com/ignitionrobotics/ign-gui/compare/ign-gui3...ign-gui2
[ign-gui-3-4]: https://github.com/ignitionrobotics/ign-gui/compare/ign-gui4...ign-gui3
[ign-gui-main]: https://github.com/ignitionrobotics/ign-gui/compare/main...ign-gui4

[ign-launch]: https://github.com/ignitionrobotics/ign-launch
[ign-launch-1-2]: https://github.com/ignitionrobotics/ign-launch/compare/ign-launch2...ign-launch1
[ign-launch-2-3]: https://github.com/ignitionrobotics/ign-launch/compare/ign-launch3...ign-launch2
[ign-launch-main]: https://github.com/ignitionrobotics/ign-launch/compare/main...ign-launch3

[ign-math]: https://github.com/ignitionrobotics/ign-math
[ign-math-main]: https://github.com/ignitionrobotics/ign-math/compare/main...ign-math6

[ign-msgs]: https://github.com/ignitionrobotics/ign-msgs
[ign-msgs-4-5]: https://github.com/ignitionrobotics/ign-msgs/compare/ign-msgs5...ign-msgs4
[ign-msgs-5-6]: https://github.com/ignitionrobotics/ign-msgs/compare/ign-msgs6...ign-msgs5
[ign-msgs-main]: https://github.com/ignitionrobotics/ign-msgs/compare/main...ign-msgs6

[ign-physics]: https://github.com/ignitionrobotics/ign-physics
[ign-physics-1-2]: https://github.com/ignitionrobotics/ign-physics/compare/ign-physics2...ign-physics1
[ign-physics-2-3]: https://github.com/ignitionrobotics/ign-physics/compare/ign-physics3...ign-physics2
[ign-physics-main]: https://github.com/ignitionrobotics/ign-physics/compare/main...ign-physics3

[ign-plugin]: https://github.com/ignitionrobotics/ign-plugin
[ign-plugin-main]: https://github.com/ignitionrobotics/ign-plugin/compare/main...ign-plugin1

[ign-rendering]: https://github.com/ignitionrobotics/ign-rendering
[ign-rendering-2-3]: https://github.com/ignitionrobotics/ign-rendering/compare/ign-rendering3...ign-rendering2
[ign-rendering-3-4]: https://github.com/ignitionrobotics/ign-rendering/compare/ign-rendering4...ign-rendering3
[ign-rendering-main]: https://github.com/ignitionrobotics/ign-rendering/compare/main...ign-rendering4

[ign-sensors]: https://github.com/ignitionrobotics/ign-sensors
[ign-sensors-2-3]: https://github.com/ignitionrobotics/ign-sensors/compare/ign-sensors3...ign-sensors2
[ign-sensors-3-4]: https://github.com/ignitionrobotics/ign-sensors/compare/ign-sensors4...ign-sensors3
[ign-sensors-main]: https://github.com/ignitionrobotics/ign-sensors/compare/main...ign-sensors4

[ign-tools]: https://github.com/ignitionrobotics/ign-tools
[ign-tools-main]: https://github.com/ignitionrobotics/ign-tools/compare/main...ign-tools1

[ign-transport]: https://github.com/ignitionrobotics/ign-transport
[ign-transport-7-8]: https://github.com/ignitionrobotics/ign-transport/compare/ign-transport8...ign-transport7
[ign-transport-8-9]: https://github.com/ignitionrobotics/ign-transport/compare/ign-transport9...ign-transport8
[ign-transport-main]: https://github.com/ignitionrobotics/ign-transport/compare/main...ign-transport9

[sdformat]: https://github.com/osrf/sdformat
[sdformat-8-9]: https://github.com/osrf/sdformat/compare/sdf9...merge_8_9_reference
[sdformat-9-10]: https://github.com/osrf/sdformat/compare/sdf10...merge_9_10_reference
[sdformat-main]: https://github.com/osrf/sdformat/compare/main...merge_10_11_reference

