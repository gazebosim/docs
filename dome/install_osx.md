# Binary Installation on MacOS Mojave (10.14)

All the Dome binaries are available in Mojave using the [homebrew package manager](https://brew.sh/).

Up to Dome's release date, the binaries should be considered unstable.

The homebrew tool can be installed using:

```bash
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

After installing the homebrew package manager, Ignition Dome can be installed running:

```bash
brew tap osrf/simulation
brew install ignition-dome
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

Head back to the [Getting started](/docs/get_started)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
brew uninstall ignition-dome
```
