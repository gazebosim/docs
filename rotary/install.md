# Gazebo Rotary

Gazebo Rotary is a Rolling Release stream of Gazebo, built nightly
from the `main` branches of every Gazebo library. It is intended for
maintainers, CI systems, and early adopters. **Not recommended for
production use** — for a stable release see [Releases](releases).

## Binary installation instructions

Binary installation is the recommended method of installing Gazebo.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * macOS: no binary distribution; see [source installation on macOS](install_osx_src)
 * Windows: no binary distribution; see [Source Installation on Windows](install_windows_src).

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

## Rotary Libraries

The Rotary collection is composed of many different Gazebo libraries. The
collection assures that all libraries are compatible and can be used together.

The best location to know the current versions of the rotary distribution is to
check [collection-rotary.yaml](https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-rotary.yaml).


## Supported platforms

These are the **officially** supported platforms:

- **Ubuntu Noble (amd64)** — nightly binary packages at
  `packages.osrfoundation.org/gazebo/ubuntu-nightly`.
- **Ubuntu Resolute (amd64)** — nightly binary packages at
  `packages.osrfoundation.org/gazebo/ubuntu-nightly`.
- **macOS** — formulae from the [`osrf/simulation`](https://github.com/osrf/homebrew-simulation) Homebrew tap.
- **Windows** — no binary; build from source.
