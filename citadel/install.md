# Citadel Installation

Citadel supports the following platforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Ignition currently only works in headless mode
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).
 * Windows 10
     * Ignition command line utilities are not yet supported.
     * All packages up to but not including `ign-gazebo` are currently building.
     * DART physics engine is not yet supported.
     * Qt (GUI functionality) is not yet supported.

## Binary installation instructions

Binary installation is the recommended method of installing Ignition.

 * [Binary Installation on Ubuntu Bionic](install_ubuntu.md)
 * [Binary Installation on MacOS Mojave (10.14)](install_osx.md)
 * [Binary Installation on Windows 10](install_windows.md)

## Source Installation instructions

Source installation is recommended for users planning on altering Ignition's source code (advanced).

 * [Source Installation on Ubuntu Bionic](install_ubuntu_src.md)
 * [Source Installation on MacOS](install_osx_src.md)
 * [Source Installation on Windows 10](install_windows_src.md)

## Citadel Libraries

The Citadel collection is composed of many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       3.x     |
|   ign-fuel-tools   |       4.x     |
|   ign-gazebo       |       3.x     |
|   ign-gui          |       3.x     |
|   ign-launch       |       2.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       5.x     |
|   ign-physics      |       2.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       3.x     |
|   ign-sensors      |       3.x     |
|   ign-tools        |       1.x     |
|   ign-transport    |       8.x     |
|   sdformat         |       9.x     |
