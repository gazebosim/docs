# Binary Installation on Ubuntu Focal

At the moment, only unstable nightly builds are available for Dome.

All of the Dome binaries are hosted in the osrfoundation repository. To install
all of them, the metapackage `ignition-dome` can be installed:

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-dome
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

Head back to the [Getting started](/docs/get_started)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
sudo apt remove ignition-dome && sudo apt autoremove
```

## Versioning in nightlies

Nightlies use the following versioning scheme:
`{current_released_version}+git{date}+${nightly_revision}r{hash}-{nightly_revision}`

* `current_released_version`: will be the latest version released available in
the changelog file of the corresponding `-release` repo. If the nightly is used
for an upcoming release, then `R-1.99.99-1` form will be used until prereleases
or final release.

* `date`: timestamp `YYYYMMDD`

* `hash`: git hash corresponding to code HEAD used in the nightly build. Used for
informational proposes.

* `nightly_revision`: revision number to apply to the nightly. It is also used to
generate a new nightly using the same same date timestamp.

