# Binary Installation on Ubuntu

Garden binaries are provided for Ubuntu Focal and Jammy. The
Garden binaries are hosted in the packages.osrfoundation.org repository.
To install all of them, the metapackage `gz-garden` can be installed.

<div class="warning">
WARNING: for gazebo-classic (eg. `gazebo11`) users: `gz-garden` cannot be
installed alongside with `gazebo11` by default. To facilitate the migration
this can be done using the instruction detailed in
<a href="https://gazebosim.org/docs/garden/install_gz11_side_by_side">Installing Gazebo11 side by side with new Gazebo</a>
</div>

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
```

Then install Gazebo Garden:


```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

All libraries should be ready to use and the `gz sim` app ready to be executed.

Head back to the [Getting started](/docs/all/getstarted)
page to start using Gazebo!


## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove gz-garden && sudo apt autoremove
```

## Troubleshooting

See [Troubleshooting](/docs/garden/troubleshooting#ubuntu)
