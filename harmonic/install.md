# Gazebo Harmonic

Gazebo Harmonic is the 8th major release of Gazebo. It is a
long-term release.

## Binary installation instructions

Binary installation is the recommended method of installing Gazebo.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on macOS](install_osx)
 * [Binary Installation on Windows](install_windows)

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
  --cmake-args -DBUILD_DOCS=ON -DBUILD_TESTING=OFF \
  --cmake-target doc
```

The generated documentation for each library can be found under:

```text
build/<package>/doxygen/html/
```

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

These are the **officially** supported platforms:

* Ubuntu Jammy on amd64
* Ubuntu Noble on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)
for the full status.
