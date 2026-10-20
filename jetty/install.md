# Gazebo Jetty

Gazebo Jetty is the 10th major release of Gazebo. It is a
long-term release.

## Binary installation instructions

Binary installation is the recommended method of installing Gazebo.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on macOS](install_osx)
 * [Binary Installation on Windows](install_windows)

## Server-only installation

A server-only package is available on Ubuntu for headless and CI environments.
It installs the Gazebo server without GUI components, avoiding Qt and extra X11
dependencies to provide a much lighter installation.

 * [Server-only Installation on Ubuntu](install_ubuntu.md#server-only-installation)

## Source Installation instructions

Source installation is recommended for users planning on altering Gazebo's source code (advanced).

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Building documentation

After completing a source installation, you can build the documentation for
the Gazebo libraries from the root of the workspace.

Install Doxygen and the documentation dependencies. On Ubuntu:

```bash
sudo apt-get install -y doxygen graphviz
```

**Note:** Building documentation for `sdformat` requires the additional
`texlive-latex-extra` package. Install the documentation dependencies with:

```bash
sudo apt-get install -y doxygen graphviz texlive-latex-extra
```

Then run:

```bash
colcon build --merge-install \
  --cmake-args -DBUILD_TESTING=OFF \
  --cmake-target doc
```

The generated documentation for each library can be found under:

```text
build/<package>/doxygen/html/
```

## Jetty Libraries

The Jetty collection is composed of many different Gazebo libraries. The
collection assures that all libraries are compatible and can be used together.

This list of library versions may change up to the release date.

| Library name       | Version       |
| ------------------ |:-------------:|
|   gz-cmake         |       5.x     |
|   gz-common        |       7.x     |
|   gz-fuel-tools    |       11.x     |
|   gz-sim           |       10.x     |
|   gz-gui           |       10.x     |
|   gz-launch        |       9.x     |
|   gz-math          |       9.x     |
|   gz-msgs          |      12.x     |
|   gz-physics       |       9.x     |
|   gz-plugin        |       4.x     |
|   gz-rendering     |       10.x     |
|   gz-sensors       |       10.x     |
|   gz-tools         |       2.x     |
|   gz-transport     |      15.x     |
|   gz-utils         |       4.x     |
|   sdformat         |      16.x     |

## Supported platforms

Jetty is planned to be [supported](releases) on the platforms below.
This list may change up to the release date.

These are the **officially** supported platforms:

* Ubuntu Noble on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/1158)
for the full status.
