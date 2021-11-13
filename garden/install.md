# Garden Fortress

Ignition Garden will be the 7th major release of Ignition. It will be a
short-term release.

Up to Garden's release date, the collection should be considered unstable.

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

## Supported platforms

Garden is planned to be [supported](/docs/all/releases) on the platforms below.
This list may change up to the release date.

### Official support

* Ubuntu Focal on amd64
* Ubuntu Jammy on amd64

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
* MacOS Catalina and BigSur
    * Binary packages may be available
    * Tested on CI (TODO which version exactly?)
    * Ignition only works in headless mode using Ogre 1.
      (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use
      `ign gazebo -s fuel.sdf` to start the server only).
* Windows 10
    * Binary packages may be available
    * Tested on CI
    * Ignition command line utilities are not supported.
    * All packages up to but not including `ign-gazebo` are currently building.
    * DART physics engine is not supported.
    * Qt (GUI functionality) is not supported.
