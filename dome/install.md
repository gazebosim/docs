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

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64

All other platforms on the table are supported at **best-effort**.

Operating System | Architecture | Packaging | CI | Functionality
---------------- | ------------ | --------- | -- | -------------
Ubuntu Bionic    | amd64        | ✅ Yes    | ✅ Yes, every pull request | ✅ All
.                | i386         | ✅ Yes    | No | ✅ All
.                | arm64        | ❓ Maybe  | No | Most low-level libraries known to work
.                | armhf        | ❓ Maybe  | No | Most low-level libraries known to work, DART physics engine not available
Ubuntu Focal     | amd64        | ✅ Yes    | ✅ Yes, every pull request | ✅ All
.                | arm64        | ❓ Maybe  | No | Most low-level libraries known to work
.                | armhf        | ❓ Maybe  | No | Most low-level libraries known to work, DART physics engine not available
Debian Buster    | amd64, i386, arm64, armhf | ❓ Maybe | No | Several libraries known to work
MacOS Catalina and BigSur | -   | ✅ Yes    | ✅ Yes, every pull request | Ignition only works in headless mode (GUI does not render; instead of using `ign gazebo fuel.sdf` command, use `ign gazebo -s fuel.sdf` to start the server only).
Windows 10       | .            | ❓ Maybe  | Some libraries tested on every pull request | Command line utilities are not supported. All packages up to but not including `ign-gazebo` are currently building. DART physics engine is not supported. Qt (GUI functionality) is not supported.

