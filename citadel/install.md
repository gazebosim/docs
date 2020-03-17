# Citadel Installation

Citadel supports the following platforms:

 * Ubuntu Bionic on amd64/i386
 * MacOS Mojave
     * Ignition currently only works in headless mode
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).

Windows support is still experimental although most of the packages should work
as expected. There are no binaries for Windows at this time. The `ign-gazebo`
package is still not available for Windows; the installation should be done from
source code. The [Source Installation on Ubuntu Bionic](install_ubuntu_src)
contains some tips for Windows.

## Binary installation instructions

Binary installation is the recommended method of installing Ignition.

 * [Binary Installation on Ubuntu Bionic](install_ubuntu)
 * [Binary Installation on MacOS Mojave (10.14)](install_osx)

## Source Installation instructions

Source installation is recommended for users planning on altering Ignition's source code (advanced).

 * [Source Installation on Ubuntu Bionic](install_ubuntu_src)
 * [Source Installation on MacOS](install_osx_src)

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
