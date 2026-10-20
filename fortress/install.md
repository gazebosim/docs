# Ignition Fortress

Ignition Fortress is the 6th major release of Ignition, and its 2nd 5-year LTS.

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

## Building documentation

After completing a source installation, you can build the documentation for
the Ignition libraries from the root of the workspace.

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

## Dockerized Development Environment instructions

Dockerized development environment is only recommended for advanced users familiar with Docker or Ignition contributors (advanced).
Can be used to contribute to Ignition without interfering with your existing installation.

 * [Dockerized Development on Ubuntu](ign_docker_env)

## Fortress Libraries

The Fortress collection is composed of many different Ignition libraries. The
collection assures that all libraries are compatible and can be used together.

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

Fortress is [supported](releases) on the platforms below.

These are the **officially** supported platforms:

* Ubuntu Bionic on amd64/i386
* Ubuntu Focal on amd64
* Ubuntu Jammy on amd64

Platforms supported at **best-effort** include arm architectures, Windows and
macOS. See
[this ticket](https://github.com/ignition-tooling/release-tools/issues/596)
for the full status.
