# Garden Installation

Up to Garden's release date, the collection should be considered unstable.

For the moment, Garden supports the following platforms:

 * Ubuntu Bionic amd64/arm64/i386 and Focal on amd64/arm64
 * MacOS Catalina and BigSur
     * Ignition currently only works in headless mode using Ogre 1
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).
 * Windows 10
     * Ignition command line utilities are not yet supported.
     * All packages up to `ign-gazebo` can be built.
     * DART physics engine is not yet supported.
     * Qt (GUI functionality) is not yet supported.

The supported platforms may change up to the release date.

## Binary installation instructions

There are no Garden binaries at the moment.

## Source Installation instructions

Source installation is recommended for users planning on altering Ignition's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Garden Libraries

The Garden collection is composed of many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

This list of library versions may change up to the release date.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       4.x     |
|   ign-fuel-tools   |       8.x     |
|   ign-gazebo       |       7.x     |
|   ign-gui          |       7.x     |
|   ign-launch       |       6.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       9.x     |
|   ign-physics      |       5.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       7.x     |
|   ign-sensors      |       7.x     |
|   ign-tools        |       1.x     |
|   ign-transport    |      12.x     |
|   ign-utils        |       1.x     |
|   sdformat         |      12.x     |
