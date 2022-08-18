# Binary Installation on MacOS

All the Fortress binaries are available in Catalina and BigSur using the
[homebrew package manager](https://brew.sh/).

The homebrew tool can be installed using:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

After installing the homebrew package manager, Ignition Fortress can be installed running:

```bash
brew tap osrf/simulation
brew install ignition-fortress
```

All libraries should be ready to use and the `ign gazebo -s` server app ready to be executed.

Head back to the [Getting started](/docs/all/getstarted)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
brew uninstall ignition-fortress
```

## Troubleshooting

See [Troubleshooting](/docs/fortress/troubleshooting#macos)
