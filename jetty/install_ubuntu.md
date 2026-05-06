# Binary Installation on Ubuntu

Jetty binaries are provided for Ubuntu Noble (24.04). The
Jetty binaries are hosted in the packages.osrfoundation.org repository.
To install all of them, the metapackage `gz-jetty` can be installed.

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Then install Gazebo Jetty:


```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-jetty
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](getstarted)
page to start using Gazebo!

(server-only-installation)=
## Server-only installation

For headless and CI environments where the GUI is not needed, the
`gz-sim10-server` package provides a lightweight alternative. It installs only
the Gazebo simulation server (`gz-sim-server`) without any GUI or Qt
dependencies, resulting in a significantly smaller installation footprint.

After configuring the repository (same as above), install the server-only
package:

```bash
sudo apt-get install gz-sim10-server
```

The server-only package includes the DART physics engine and the core
simulation server plugins. To start a headless simulation:

```bash
/usr/libexec/gz/sim10/gz-sim-server <world_file.sdf>
```

:::{note}
The server-only binary is installed at `/usr/libexec/gz/sim10/gz-sim-server`,
which is not in the `PATH` by default. The regular `gz sim -s` command is not
available in the server-only installation since it requires the full
`gz-sim10-cli` package.
:::

## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove gz-jetty && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#ubuntu)
