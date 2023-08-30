# Binary Installation on Ubuntu

Harmonic pre-release binaries are provided for Ubuntu Jammy. The
Harmonic binaries are hosted in the packages.osrfoundation.org repository.
To install all of them, the metapackage `gz-harmonic` can be installed.

**WARNING:** `gz-harmonic` cannot be installed alongside gazebo-classic (eg. `gazebo11`) since both use the `gz` command line tool. Trying to install `gz-harmonic` on a system that already has gazebo-classic installed from binaries will cause gazebo-classic and its dependencies to be uninstalled. Currently, the workarounds for this are to install from source or to use Docker [`gazebo-classic`](https://hub.docker.com/_/gazebo) so they are not installed side-by-side on the same system.

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```

Then install Gazebo Harmonic:


```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-prerelease $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-prerelease.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!


## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove gz-harmonic && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](/docs/harmonic/troubleshooting#ubuntu)
