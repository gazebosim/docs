# Gazebo Ionic

Up to Ionic's release date, the collection should be considered unstable.

Gazebo Ionic will be the 8th major release of Gazebo. It will be a
long-term release.

## Binary installation instructions

Binary installation is the recommended method of installing Gazebo.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on macOS](install_osx)
 * [Binary Installation on Windows](install_windows)

## Source Installation instructions

Source installation is recommended for users planning on altering Gazebo's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Ionic Libraries

The Ionic collection is composed of many different Gazebo libraries. The
collection assures that all libraries are compatible and can be used together.

This list of library versions may change up to the release date.

| Library name       | Version       |
| ------------------ |:-------------:|
|   gz-cmake         |       4.x     |
|   gz-common        |       6.x     |
|   gz-fuel-tools    |       10.x     |
|   gz-sim           |       9.x     |
|   gz-gui           |       9.x     |
|   gz-launch        |       8.x     |
|   gz-math          |       8.x     |
|   gz-msgs          |      11.x     |
|   gz-physics       |       8.x     |
|   gz-plugin        |       3.x     |
|   gz-rendering     |       9.x     |
|   gz-sensors       |       9.x     |
|   gz-tools         |       2.x     |
|   gz-transport     |      14.x     |
|   gz-utils         |       3.x     |
|   sdformat         |      15.x     |

## Supported platforms

Ionic is planned to be [supported](/docs/all/releases) on the platforms below.
This list may change up to the release date.

These are the **officially** supported platforms:

* Ubuntu Jammy on amd64
* Ubuntu Noble on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)
for the full status.
