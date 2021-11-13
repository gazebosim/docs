# Ignition Dome

Ignition Dome is the 4th major release of Ignition. It's a short-term support.

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

## Supported platforms

Dome is [supported](/docs/all/releases) on the platforms below.

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
* MacOS Mojave and Catalina
    * Binary packages may be available
    * Tested on CI (TODO which version exactly?)
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
