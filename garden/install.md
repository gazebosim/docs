# Gazebo Garden

Gazebo Garden will be the 7th major release of Gazebo. It will be a
short-term release.

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

## Garden Libraries

The Garden collection is composed of many different Gazebo libraries. The
collection assures that all libraries are compatible and can be used together.

This list of library versions may change up to the release date.

| Library name       | Version       |
| ------------------ |:-------------:|
|   gz-cmake         |       3.x     |
|   gz-common        |       5.x     |
|   gz-fuel-tools    |       8.x     |
|   gz-sim           |       7.x     |
|   gz-gui           |       7.x     |
|   gz-launch        |       6.x     |
|   gz-math          |       7.x     |
|   gz-msgs          |       9.x     |
|   gz-physics       |       6.x     |
|   gz-plugin        |       2.x     |
|   gz-rendering     |       7.x     |
|   gz-sensors       |       7.x     |
|   gz-tools         |       2.x     |
|   gz-transport     |      12.x     |
|   gz-utils         |       2.x     |
|   sdformat         |      13.x     |

## Supported platforms

Garden is planned to be [supported](/docs/all/releases) on the platforms below.
This list may change up to the release date.

These are the **officially** supported platforms:

* Ubuntu Focal on amd64
* Ubuntu Jammy on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)
for the full status.

## Migration Guide

Gazebo Garden is the first major release that [uses the Gazebo brand instead of Ignition](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356).
So if you are upgrading from a prior release (e.g. Fortress), you will need to do some steps to migrate your packages.

- [Guide for Migrating From Ignition](migration_from_ignition)

