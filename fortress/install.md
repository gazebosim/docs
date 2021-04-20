# Fortress Installation

Until the official release, Fortress can be compiled from source or installed
from nightly debian packages on Linux.

Fortress supports the following platforms:

 * Ubuntu Bionic amd64 and Focal on amd64
 * MacOS Mojave and Catalina
     * Ignition currently only works in headless mode
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).
 * Windows 10
     * Ignition command line utilities are not yet supported.
     * All packages up to `ign-gazebo` can be built.
     * DART physics engine is not yet supported.
     * Qt (GUI functionality) is not yet supported.

## Binary installation instructions

Binary installation is the recommended method of installing Ignition.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on macOS](install_osx)
 * [Binary Installation on Windows](install_windows)

## Source Installation instructions

Source installation is recommended for users planning on altering Ignition's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Fortress Libraries

The Fortress collection is composed of many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       4.x     |
|   ign-fuel-tools   |       7.x     |
|   ign-gazebo       |       6.x     |
|   ign-gui          |       6.x     |
|   ign-launch       |       5.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       8.x     |
|   ign-physics      |       5.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       6.x     |
|   ign-sensors      |       6.x     |
|   ign-tools        |       1.x     |
|   ign-transport    |      11.x     |
|   ign-utils        |       1.x     |
|   sdformat         |      12.x     |
