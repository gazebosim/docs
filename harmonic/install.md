# Gazebo Harmonic

Up to Harmonic's release date, the collection should be considered unstable.

Gazebo Harmonic will be the 8th major release of Gazebo. It will be a
long-term release.

## Binary installation instructions

There are no Harmonic binaries at the moment

## Source Installation instructions

Source installation is recommended for users planning on altering Gazebo's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Harmonic Libraries

The Harmonic collection is composed of many different Gazebo libraries. The
collection assures that all libraries are compatible and can be used together.

This list of library versions may change up to the release date.

| Library name       | Version       |
| ------------------ |:-------------:|
|   gz-cmake         |       3.x     |
|   gz-common        |       5.x     |
|   gz-fuel-tools    |       9.x     |
|   gz-sim           |       8.x     |
|   gz-gui           |       8.x     |
|   gz-launch        |       7.x     |
|   gz-math          |       7.x     |
|   gz-msgs          |      10.x     |
|   gz-physics       |       7.x     |
|   gz-plugin        |       2.x     |
|   gz-rendering     |       8.x     |
|   gz-sensors       |       8.x     |
|   gz-tools         |       2.x     |
|   gz-transport     |      13.x     |
|   gz-utils         |       2.x     |
|   sdformat         |      14.x     |

## Supported platforms

Harmonic is planned to be [supported](/docs/all/releases) on the platforms below.
This list may change up to the release date.

These are the **officially** supported platforms:

* Ubuntu Jammy on amd64
* Ubuntu 24.04 on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)
for the full status.
