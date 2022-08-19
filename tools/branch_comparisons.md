# Branch comparisons

These links take to comparisons between branches.

If a cell in the table is `-`, this means that the two Gazebo releases use
the same version of this Gazebo library. So, for example, `gz-cmake` has a
`-` for C ➡️  E because Citadel and Fortress use `ign-cmake2`.

When the comparison link says "There isn’t anything to compare," this means
no pull requests need to be forward-ported.

Otherwise, those branches could use a forward-port pull request.
See the instructions on the
[contributing guide](https://gazebosim.org/docs/all/contributing#process).

* **C**: Citadel
* **E**: Edifice
* **F**: Fortress
* **G**: Garden
* **main**: main branch

Library                        | C ➡️  F                        |  F ➡️  G                        | G ➡️  main
------------------------------ | ----------------------------- | ------------------------------ | ---------------------------------
[gz-cmake][gz-cmake]           | -                             | [2 ➡️  3][gz-cmake-2-3]         | [3 ➡️  main][gz-cmake-main]
[gz-common][gz-common]         | [3 ➡️  4][ign-common-3-4]      | [4 ➡️  5][gz-common-4-5]        | [5 ➡️  main][gz-common-main]
[gz-fuel-tools][gz-fuel-tools] | [4 ➡️  7][ign-fuel-tools-4-7]  | [7 ➡️  8][gz-fuel-tools-7-8]    | [8 ➡️  main][gz-fuel-tools-main]
[gz-sim][gz-sim]               | [3 ➡️  6][ign-gazebo-3-6]      | [6 ➡️  7][gz-sim-6-7]           | [7 ➡️  main][gz-sim-main]
[gz-gui][gz-gui]               | [3 ➡️  6][ign-gui-3-6]         | [6 ➡️  7][gz-gui-6-7]           | [7 ➡️  main][gz-gui-main]
[gz-launch][gz-launch]         | [2 ➡️  5][ign-launch-2-5]      | [5 ➡️  6][gz-launch-5-6]        | [6 ➡️  main][gz-launch-main]
[gz-math][gz-math]             | -                             | [6 ➡️  7][gz-math-6-7]          | [7 ➡️  main][gz-math-main]
[gz-msgs][gz-msgs]             | [5 ➡️  8][ign-msgs-5-8]        | [8 ➡️  9][gz-msgs-8-9]          | [9 ➡️  main][gz-msgs-main]
[gz-physics][gz-physics]       | [2 ➡️  5][ign-physics-2-5]     | [5 ➡️  6][gz-physics-5-6]       | [6 ➡️  main][gz-physics-main]
[gz-plugin][gz-plugin]         | -                             | [1 ➡️  2][gz-plugin-1-2]        | [2 ➡️  main][gz-plugin-main]
[gz-rendering][gz-rendering]   | [3 ➡️  6][ign-rendering-3-6]   | [6 ➡️  7][gz-rendering-6-7]     | [7 ➡️  main][gz-rendering-main]
[gz-sensors][gz-sensors]       | [3 ➡️  6][ign-sensors-3-6]     | [6 ➡️  7][gz-sensors-6-7]       | [7 ➡️  main][gz-sensors-main]
[gz-tools][gz-tools]           | -                             | [1 ➡️  2][gz-tools-1-2]         | [2 ➡️  main][gz-tools-main]
[gz-transport][gz-transport]   | [8 ➡️  11][ign-transport-8-11] | [11 ➡️  12][gz-transport-11-12] | [12 ➡️  main][gz-transport-main]
[gz-utils][gz-utils]           | -                             | [1 ➡️  2][gz-utils-1-2]         | [2 ➡️  main][gz-utils-main]
[sdformat][sdformat]           | [9 ➡️  12][sdformat-9-12]      | [12 ➡️  13][sdformat-12-13]     | [13 ➡️  main][sdformat-main]

[gz-cmake]: https://github.com/gazebosim/gz-cmake
[gz-cmake-2-3]: https://github.com/gazebosim/gz-cmake/compare/gz-cmake3...ign-cmake2
[gz-cmake-main]: https://github.com/gazebosim/gz-cmake/compare/main...gz-cmake3

[gz-common]: https://github.com/gazebosim/gz-common
[ign-common-3-4]: https://github.com/gazebosim/gz-common/compare/ign-common4...ign-common3
[gz-common-4-5]: https://github.com/gazebosim/gz-common/compare/gz-common5...ign-common4
[gz-common-main]: https://github.com/gazebosim/gz-common/compare/main...gz-common5

[gz-fuel-tools]: https://github.com/gazebosim/gz-fuel-tools
[ign-fuel-tools-4-7]: https://github.com/gazebosim/gz-fuel-tools/compare/ign-fuel-tools7...ign-fuel-tools4
[gz-fuel-tools-7-8]: https://github.com/gazebosim/gz-fuel-tools/compare/gz-fuel-tools8...ign-fuel-tools7
[gz-fuel-tools-main]: https://github.com/gazebosim/gz-fuel-tools/compare/main...gz-fuel-tools8

[gz-sim]: https://github.com/gazebosim/gz-sim
[ign-gazebo-3-6]: https://github.com/gazebosim/gz-sim/compare/ign-gazebo6...ign-gazebo3
[gz-sim-6-7]: https://github.com/gazebosim/gz-sim/compare/gz-sim7...ign-gazebo6
[gz-sim-main]: https://github.com/gazebosim/gz-sim/compare/main...gz-sim7

[gz-gui]: https://github.com/gazebosim/gz-gui
[ign-gui-3-6]: https://github.com/gazebosim/gz-gui/compare/ign-gui6...ign-gui3
[gz-gui-6-7]: https://github.com/gazebosim/gz-gui/compare/gz-gui7...ign-gui6
[gz-gui-main]: https://github.com/gazebosim/gz-gui/compare/main...gz-gui7

[gz-launch]: https://github.com/gazebosim/gz-launch
[ign-launch-2-5]: https://github.com/gazebosim/gz-launch/compare/ign-launch5...ign-launch2
[gz-launch-5-6]: https://github.com/gazebosim/gz-launch/compare/gz-launch6...ign-launch5
[gz-launch-main]: https://github.com/gazebosim/gz-launch/compare/main...gz-launch6

[gz-math]: https://github.com/gazebosim/gz-math
[gz-math-6-7]: https://github.com/gazebosim/gz-math/compare/gz-math7...ign-math6
[gz-math-main]: https://github.com/gazebosim/gz-math/compare/main...gz-math7

[gz-msgs]: https://github.com/gazebosim/gz-msgs
[ign-msgs-5-8]: https://github.com/gazebosim/gz-msgs/compare/ign-msgs8...ign-msgs5
[gz-msgs-8-9]: https://github.com/gazebosim/gz-msgs/compare/gz-msgs9...ign-msgs8
[gz-msgs-main]: https://github.com/gazebosim/gz-msgs/compare/main...gz-msgs9

[gz-physics]: https://github.com/gazebosim/gz-physics
[ign-physics-2-5]: https://github.com/gazebosim/gz-physics/compare/ign-physics5...ign-physics2
[gz-physics-5-6]: https://github.com/gazebosim/gz-physics/compare/gz-physics6...ign-physics5
[gz-physics-main]: https://github.com/gazebosim/gz-physics/compare/main...ign-physics6

[gz-plugin]: https://github.com/gazebosim/gz-plugin
[gz-plugin-1-2]: https://github.com/gazebosim/gz-plugin/compare/gz-plugin2...ign-plugin1
[gz-plugin-main]: https://github.com/gazebosim/gz-plugin/compare/main...gz-plugin2

[gz-rendering]: https://github.com/gazebosim/gz-rendering
[ign-rendering-3-6]: https://github.com/gazebosim/gz-rendering/compare/ign-rendering6...ign-rendering3
[gz-rendering-6-7]: https://github.com/gazebosim/gz-rendering/compare/gz-rendering7...ign-rendering6
[gz-rendering-main]: https://github.com/gazebosim/gz-rendering/compare/main...gz-rendering7

[gz-sensors]: https://github.com/gazebosim/gz-sensors
[ign-sensors-3-6]: https://github.com/gazebosim/gz-sensors/compare/ign-sensors6...ign-sensors3
[gz-sensors-6-7]: https://github.com/gazebosim/gz-sensors/compare/gz-sensors7...ign-sensors6
[gz-sensors-main]: https://github.com/gazebosim/gz-sensors/compare/main...gz-sensors7

[gz-tools]: https://github.com/gazebosim/gz-tools
[gz-tools-1-2]: https://github.com/gazebosim/gz-tools/compare/gz-tools2...ign-tools1
[gz-tools-main]: https://github.com/gazebosim/gz-tools/compare/main...gz-tools2

[gz-transport]: https://github.com/gazebosim/gz-transport
[ign-transport-8-11]: https://github.com/gazebosim/gz-transport/compare/ign-transport11...ign-transport8
[gz-transport-11-12]: https://github.com/gazebosim/gz-transport/compare/gz-transport12...ign-transport11
[gz-transport-main]: https://github.com/gazebosim/gz-transport/compare/main...gz-transport12

[gz-utils]: https://github.com/gazebosim/gz-utils
[gz-utils-1-2]: https://github.com/gazebosim/gz-utils/compare/gz-utils2...ign-utils1
[gz-utils-main]: https://github.com/gazebosim/gz-utils/compare/main...gz-utils2

[sdformat]: https://github.com/osrf/sdformat
[sdformat-9-12]: https://github.com/osrf/sdformat/compare/sdf12...sdf9
[sdformat-12-13]: https://github.com/osrf/sdformat/compare/sdf13...sdf12
[sdformat-main]: https://github.com/osrf/sdformat/compare/main...sdf13
