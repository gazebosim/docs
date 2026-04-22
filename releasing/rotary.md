# Rotary release

**Rotary** is a rolling release stream of Gazebo built continuously from the
`main` branches of every Gazebo library. Unlike the dated codename releases
(Fortress, Harmonic, Jetty, …), rotary has no fixed version: it is built
every night. It is intended for Gazebo maintainers, continuous
integration systems, and early adopters who want to test unreleased
changes before they ship in a named collection.

:::{warning}
Rotary is a rolling, unstable stream. Breakage is expected, and versions
may change under you on every `apt upgrade` or `brew upgrade`. Rotary is
**not recommended for production use**. New Gazebo users should install
one of the binary releases listed in [Releases](../releases.md) instead.
:::

## Package naming

Wherever a Gazebo package name contains `gz-`, a `rotary-` infix is
injected to produce the rotary alias. The aliases point to unversioned
upstream packages, so installing a rotary alias always pulls whatever is
currently being built from `main`.

### Ubuntu / Debian


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

## Installing rotary on Ubuntu

:::{note}
Rotary binaries are published into the existing **nightly** apt repository
at `http://packages.osrfoundation.org/gazebo/{ubuntu,debian}-nightly`, so
the versioning scheme documented in
[Debian/Ubuntu versioning in nightly and prerelease binaries](versioning_pre_nightly.md)
applies to rotary unchanged.
:::

First install the prerequisite tools:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Add the OSRF GPG key and configure the **nightly** apt repository (which
hosts rotary packages):

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-nightly.list > /dev/null
sudo apt-get update
```

Then install the rotary metapackage:

```bash
sudo apt-get install gz-rotary
```

All rotary libraries should be ready to use and the `gz sim` app ready to
run.

### Uninstalling rotary

```bash
sudo apt remove gz-rotary && sudo apt autoremove
```

## Installing rotary on macOS (Homebrew)

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

The initial rollout of rotary brew formulae is tracked in
[osrf/homebrew-simulation#3287](https://github.com/osrf/homebrew-simulation/pull/3287).

## Windows

There is currently no binary distribution of rotary on Windows. Windows
users who want to run rotary need to build it from source.

The existing [Source Installation on Windows 10 or 11](../install_windows_src.md)
guide for Jetty describes the full toolchain (Visual Studio, Pixi,
`colcon`). The same procedure applies to rotary — the only change is
the `vcs import` step, which must point at `collection-rotary.yaml` from
[gazebo-tooling/gazebodistro](https://github.com/gazebo-tooling/gazebodistro)
instead of the Jetty collection file:

```bash
vcs import --input https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-rotary.yaml src/
```

After the import, continue with the "Building the Gazebo Libraries" step
of the Jetty Windows source guide unchanged.
