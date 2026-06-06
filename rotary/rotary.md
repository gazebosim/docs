# Rotary release

**Rotary** is a rolling release stream of Gazebo built continuously from the
`main` branches of every Gazebo library. Unlike the dated codename releases
(Fortress, Harmonic, Jetty, ...), Rotary has no fixed version: it is built
every night. It is intended for Gazebo maintainers, continuous
integration systems, and early adopters who want to test unreleased
changes before they ship in a named collection.

:::{warning}
Rotary is a rolling, unstable stream. Breakage is expected, and versions
may change under you on every `apt upgrade` or `brew upgrade`. Rotary is
**not recommended for production use**. New Gazebo users should install
one of the binary releases listed in [Releases](releases) instead.
:::

## Package naming

Wherever a Gazebo package name contains `gz-`, a `rotary-` infix is
injected to produce the Rotary alias. The aliases point to unversioned
upstream packages, so installing a Rotary alias always pulls whatever is
currently being built from `main`.

### Ubuntu

The package names in the table below are examples illustrating the
aliasing rule:


| Regular package        | Rotary alias                    |
|------------------------|---------------------------------|
| `libgz-cmake-dev`      | `libgz-rotary-cmake-dev`        |
| `gz-plugin-cli`        | `gz-rotary-plugin-cli`          |
| `python3-gz-math`      | `python3-gz-rotary-math`        |
| `libsdformat-dev` (*)  | `libgz-rotary-sdformat-dev`     |

(*) sdformat is a special case: its package names do not contain `gz-`,
so the rotary alias adds a `gz-rotary-` prefix.

### Homebrew (macOS)

| Regular formula | Rotary alias        |
|-----------------|---------------------|
| `gz-mathN`      | `gz-rotary-math`    |
| `sdformatN`     | `gz-rotary-sdformat`|

### Umbrella metapackage

On both Ubuntu and Homebrew, the `gz-rotary` metapackage pulls in the full
set of rotary libraries, mirroring how `gz-jetty` or `gz-harmonic` work for
named collections.

## Installing Rotary on Ubuntu

:::{note}
Rotary binaries are published into the existing **nightly** apt repository
at `http://packages.osrfoundation.org/gazebo/ubuntu-nightly`, so
the versioning scheme documented in
[Ubuntu versioning in nightly and prerelease binaries](releasing/versioning_pre_nightly.md)
applies to Rotary unchanged.
:::

First install the prerequisite tools:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Add the OSRF GPG key and configure the **nightly** apt repository (which
hosts Rotary packages) that needs the stable repository also to work:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-nightly.list > /dev/null
sudo apt-get update
```

Then install the Rotary metapackage:

```bash
sudo apt-get install gz-rotary
```

All Rotary libraries should be ready to use and the `gz sim` app ready to
run.

### Uninstalling Rotary

```bash
sudo apt remove gz-rotary && sudo apt autoremove
```

## Installing Rotary on macOS (Homebrew)

Rotary formulae are published through the
[`osrf/simulation`](https://github.com/osrf/homebrew-simulation) tap:

```bash
brew tap osrf/simulation
brew install gz-rotary
```

Per-library formulae are also available using the `gz-rotary-<library>`
naming scheme, for example:

```bash
brew install gz-rotary-math gz-rotary-sim
```

The initial rollout of Rotary brew formulae is tracked in
[osrf/homebrew-simulation#3287](https://github.com/osrf/homebrew-simulation/pull/3287).

## Windows

There is currently no binary distribution of Rotary on Windows. Windows
users who want to run Rotary need to build it from source.

The [Source Installation on Windows 10 or 11](install_windows_src) guide
describes the full toolchain (Visual Studio, Pixi, `colcon`) and uses the
Rotary collection file, `collection-rotary.yaml`, from
[gazebo-tooling/gazebodistro](https://github.com/gazebo-tooling/gazebodistro)
in its `vcs import` step:

```bash
vcs import --input https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-rotary.yaml src/
```
