# Binary Installation on Ubuntu

Citadel binaries are provided for Ubuntu Bionic and Focal. All of the Citadel
binaries are hosted in the osrfoundation repository. To install all of them,
the metapackage `ignition-citadel` can be installed.

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

Then install Ignition Citadel:


```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
curl https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-citadel
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

Head back to the [Getting started](/docs/all/getstarted)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove ignition-citadel && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](/docs/citadel/troubleshooting#ubuntu)
