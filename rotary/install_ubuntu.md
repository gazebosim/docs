# Binary Installation on Ubuntu

Rotary binaries are published for Ubuntu Noble (24.04) and Ubuntu
Resolute (26.04) via the nightly
packages repository at `packages.osrfoundation.org`.
To install all of them, the metapackage `gz-rotary` can be installed.

:::{warning}
Rotary is a rolling, unstable stream. Packages are updated nightly and breakage
is expected. For production use, install a named release such as
[Jetty](https://gazebosim.org/docs/jetty) instead.
:::

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Then install Gazebo Rotary:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-nightly.list > /dev/null
sudo apt-get update
sudo apt-get install gz-rotary
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](getstarted)
page to start using Gazebo!

(server-only-installation)=
## Server-only installation

For headless and CI environments where the GUI is not needed, the
`gz-sim-server` package provides a lightweight alternative. It installs only
the Gazebo simulation server (`gz-sim-server`) without any GUI or Qt
dependencies, resulting in a significantly smaller installation footprint.

After configuring the repository (same as above), install the server-only
package:

```bash
sudo apt-get install gz-sim-server
```

:::{note}
The server-only package is an exception to the `gz-rotary-` aliasing rule:
there is no `gz-rotary-sim-server` alias. The unversioned `gz-sim-server`
package is built directly from `main`, which is exactly what Rotary tracks.
:::

The server-only package includes the DART physics engine and the core
simulation server plugins. To start a headless simulation:

```bash
/usr/libexec/gz/sim/gz-sim-server <world_file.sdf>
```

:::{note}
The server-only binary is installed at `/usr/libexec/gz/sim/gz-sim-server`,
which is not in the `PATH` by default. The regular `gz sim -s` command is not
available in the server-only installation since it requires the full
`gz-sim-cli` package.
:::


## Uninstalling binary install

If you need to uninstall Gazebo Rotary or switch to a source-based install,
run the following command:

```bash
sudo apt remove gz-rotary && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#ubuntu)
