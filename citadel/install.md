# Ignition Citadel

Ignition Citadel is the 3rd major release of Ignition, and its 1st 5-year-LTS.

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

## Supported platforms

Citadel is [supported](/docs/all/releases) on the platforms below.

### Official support

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64

### Best-effort support

* Ubuntu Bionic on arm64
    * Binary packages may be available
    * Not tested on CI
    * Most low-level libraries known to work
    * TODO: what is know not to work?
* Ubuntu Focal on arm64
    * Binary packages may be available
    * Not tested on CI
    * Most low-level libraries known to work
    * TODO: what is know not to work?
* Ubuntu Bionic on armhf
    * Binary packages may be available
    * Not tested on CI
    * Most low-level libraries known to work
    * DART physics engine not available
* Ubuntu Focal on armhf
    * Binary packages may be available
    * Not tested on CI
    * Most low-level libraries known to work
    * DART physics engine not available
* Debian Buster on amd64, i386, arm64 and armhf
    * Binary packages may be available
    * Not tested on CI
    * Most low-level libraries known to work
    * TODO: what is know not to work?
* MacOS Mojave
    * Binary packages may be available
    * Tested on CI
    * Ignition only works in headless mode
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).
* Windows 10
    * Binary packages may be available
    * Some libraries tested on CI
    * Ignition command line utilities are not supported.
    * All packages up to but not including `ign-gazebo` are currently building.
    * DART physics engine is not supported.
    * Qt (GUI functionality) is not supported.
