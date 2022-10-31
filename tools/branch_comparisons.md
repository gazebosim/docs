# Branch comparisons

These links take to comparisons between branches.

If a cell in the table is `-`, this means that the two Gazebo releases use
the same version of this Gazebo library. So, for example, `ign-cmake` has a
`-` for C ➡️  E because Citadel and Edifice use `ign-cmake2`.

When the comparison link says "There isn’t anything to compare," this means
no pull requests need to be forward-ported.

Otherwise, those branches could use a forward-port pull request.
See the instructions on the
[contributing guide](https://gazebosim.org/docs/all/contributing#process).

* **C**: Citadel
* **E**: Edifice
* **F**: Fortress
* **main**: main branch

Library                          | C ➡️  F                        | F ➡️  main
-------------------------------- | ----------------------------- | ---------------------------------
[ign-cmake][ign-cmake]           | -                             | [2 ➡️  main][ign-cmake-main]
[ign-common][ign-common]         | [3 ➡️  4][ign-common-3-4]      | [4 ➡️  main][ign-common-main]
[ign-fuel-tools][ign-fuel-tools] | [4 ➡️  7][ign-fuel-tools-4-7]  | [7 ➡️  main][ign-fuel-tools-main]
[ign-gazebo][ign-gazebo]         | [3 ➡️  6][ign-gazebo-3-6]      | [6 ➡️  main][ign-gazebo-main]
[ign-gui][ign-gui]               | [3 ➡️  6][ign-gui-3-6]         | [6 ➡️  main][ign-gui-main]
[ign-launch][ign-launch]         | [2 ➡️  5][ign-launch-2-5]      | [5 ➡️  main][ign-launch-main]
[ign-math][ign-math]             | -                             | [6 ➡️  main][ign-math-main]
[ign-msgs][ign-msgs]             | [5 ➡️  8][ign-msgs-5-8]        | [8 ➡️  main][ign-msgs-main]
[ign-physics][ign-physics]       | [2 ➡️  5][ign-physics-2-5]     | [5 ➡️  main][ign-physics-main]
[ign-plugin][ign-plugin]         | -                             | [1 ➡️  main][ign-plugin-main]
[ign-rendering][ign-rendering]   | [3 ➡️  6][ign-rendering-3-6]   | [6 ➡️  main][ign-rendering-main]
[ign-sensors][ign-sensors]       | [3 ➡️  6][ign-sensors-3-6]     | [6 ➡️  main][ign-sensors-main]
[ign-tools][ign-tools]           | -                             | [1 ➡️  main][ign-tools-main]
[ign-transport][ign-transport]   | [8 ➡️  11][ign-transport-8-11] | [11 ➡️  main][ign-transport-main]
[ign-utils][ign-utils]           | -                             | [1 ➡️  main][ign-utils-main]
[sdformat][sdformat]             | [9 ➡️  12][sdformat-9-12]      | [12 ➡️  main][sdformat-main]

[ign-cmake]: https://github.com/gazebosim/gz-cmake
[ign-cmake-main]: https://github.com/gazebosim/gz-cmake/compare/main...ign-cmake2

[ign-common]: https://github.com/gazebosim/gz-common
[ign-common-3-4]: https://github.com/gazebosim/gz-common/compare/ign-common4...ign-common3
[ign-common-main]: https://github.com/gazebosim/gz-common/compare/main...ign-common4

[ign-fuel-tools]: https://github.com/gazebosim/gz-fuel-tools
[ign-fuel-tools-4-7]: https://github.com/gazebosim/gz-fuel-tools/compare/ign-fuel-tools7...ign-fuel-tools4
[ign-fuel-tools-main]: https://github.com/gazebosim/gz-fuel-tools/compare/main...ign-fuel-tools7

[ign-gazebo]: https://github.com/gazebosim/gz-sim
[ign-gazebo-3-6]: https://github.com/gazebosim/gz-sim/compare/ign-gazebo6...ign-gazebo3
[ign-gazebo-main]: https://github.com/gazebosim/gz-sim/compare/main...ign-gazebo6

[ign-gui]: https://github.com/gazebosim/gz-gui
[ign-gui-3-6]: https://github.com/gazebosim/gz-gui/compare/ign-gui6...ign-gui3
[ign-gui-main]: https://github.com/gazebosim/gz-gui/compare/main...ign-gui6

[ign-launch]: https://github.com/gazebosim/gz-launch
[ign-launch-2-5]: https://github.com/gazebosim/gz-launch/compare/ign-launch5...ign-launch2
[ign-launch-main]: https://github.com/gazebosim/gz-launch/compare/main...ign-launch5

[ign-math]: https://github.com/gazebosim/gz-math
[ign-math-main]: https://github.com/gazebosim/gz-math/compare/main...ign-math6

[ign-msgs]: https://github.com/gazebosim/gz-msgs
[ign-msgs-5-8]: https://github.com/gazebosim/gz-msgs/compare/ign-msgs8...ign-msgs5
[ign-msgs-main]: https://github.com/gazebosim/gz-msgs/compare/main...ign-msgs8

[ign-physics]: https://github.com/gazebosim/gz-physics
[ign-physics-2-5]: https://github.com/gazebosim/gz-physics/compare/ign-physics5...ign-physics2
[ign-physics-main]: https://github.com/gazebosim/gz-physics/compare/main...ign-physics5

[ign-plugin]: https://github.com/gazebosim/gz-plugin
[ign-plugin-main]: https://github.com/gazebosim/gz-plugin/compare/main...ign-plugin1

[ign-rendering]: https://github.com/gazebosim/gz-rendering
[ign-rendering-3-6]: https://github.com/gazebosim/gz-rendering/compare/ign-rendering6...ign-rendering3
[ign-rendering-main]: https://github.com/gazebosim/gz-rendering/compare/main...ign-rendering6

[ign-sensors]: https://github.com/gazebosim/gz-sensors
[ign-sensors-3-6]: https://github.com/gazebosim/gz-sensors/compare/ign-sensors6...ign-sensors3
[ign-sensors-main]: https://github.com/gazebosim/gz-sensors/compare/main...ign-sensors6

[ign-tools]: https://github.com/gazebosim/gz-tools
[ign-tools-main]: https://github.com/gazebosim/gz-tools/compare/main...ign-tools1

[ign-transport]: https://github.com/gazebosim/gz-transport
[ign-transport-8-11]: https://github.com/gazebosim/gz-transport/compare/ign-transport11...ign-transport8
[ign-transport-main]: https://github.com/gazebosim/gz-transport/compare/main...ign-transport11

[ign-utils]: https://github.com/gazebosim/gz-utils
[ign-utils-main]: https://github.com/gazebosim/gz-utils/compare/main...ign-utils1

[sdformat]: https://github.com/osrf/sdformat
[sdformat-9-12]: https://github.com/osrf/sdformat/compare/sdf12...sdf9
[sdformat-main]: https://github.com/osrf/sdformat/compare/main...sdf12
