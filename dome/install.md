# Dome Installation

Dome supports the following platforms:

 * Ubuntu Bionic amd64/arm64/i386 and Focal on amd64/arm64
 * MacOS Mojave and Catalina
     * Ignition currently only works in headless mode
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this time. The `ign-gazebo`
package is still not available for Windows; the installation should be done from
source code. The [Source Installation on Ubuntu](install_ubuntu_src)
contains some tips for Windows.

## Binary installation instructions

Binary builds are available for Linux and macOS.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on MacOS](install_osx)

## Source Installation instructions

Source installation is recommended for users planning on altering Ignition's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on MacOS](install_osx_src)

## Dome Libraries

The Dome collection is composed of many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

| Library name       | Version       |
| ------------------ |:-------------:|
|   ign-cmake        |       2.x     |
|   ign-common       |       3.x     |
|   ign-fuel-tools   |       5.x     |
|   ign-gazebo       |       4.x     |
|   ign-gui          |       4.x     |
|   ign-launch       |       3.x     |
|   ign-math         |       6.x     |
|   ign-msgs         |       6.x     |
|   ign-physics      |       3.x     |
|   ign-plugin       |       1.x     |
|   ign-rendering    |       4.x     |
|   ign-sensors      |       4.x     |
|   ign-tools        |       1.x     |
|   ign-transport    |       9.x     |
|   sdformat         |      10.x     |
