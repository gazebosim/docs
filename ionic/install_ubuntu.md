# Binary Installation on Ubuntu

Ionic binaries are provided for Ubuntu Jammy (22.04) and Ubuntu Noble (24.04). The
Ionic binaries are hosted in the packages.osrfoundation.org repository.
To install all of them, the metapackage `gz-ionic` can be installed.

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

Then install Gazebo Ionic:


```bash
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-ionic
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!


## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove gz-ionic && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](/docs/ionic/troubleshooting#ubuntu)
