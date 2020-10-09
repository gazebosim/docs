# Binary Installation on Ubuntu

Dome binaries are provided for Ubuntu Bionic and Focal. All of the Dome
binaries are hosted in the osrfoundation repository. To install all of them,
the metapackage `ignition-dome` can be installed:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-dome
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

Head back to the [Getting started](/docs/all/get_started)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove ignition-dome && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](troubleshooting)
