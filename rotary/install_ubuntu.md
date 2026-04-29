# Binary Installation on Ubuntu

Rotary binaries are published for Ubuntu Noble (24.04) via the nightly
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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-nightly.list > /dev/null
sudo apt-get update
sudo apt-get install gz-rotary
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](getstarted)
page to start using Gazebo!

## Uninstalling binary install

If you need to uninstall Gazebo Rotary or switch to a source-based install,
run the following command:

```bash
sudo apt remove gz-rotary && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#ubuntu)
