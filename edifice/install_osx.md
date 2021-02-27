# Binary Installation on MacOS

All the Edifice binaries are available in Mojave and Catalina using the
[homebrew package manager](https://brew.sh/).

Up to Edifice's release date, the binaries should be considered unstable.

The homebrew tool can be installed using:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

After installing the homebrew package manager, Ignition Edifice can be installed running:

```bash
brew tap osrf/simulation
brew install ignition-edifice
```

All libraries should be ready to use and the `ign gazebo` app ready to be executed.

Head back to the [Getting started](/docs/all/get_started)
page to start using Ignition!

## Uninstalling binary install

If you need to uninstall Ignition or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
brew uninstall ignition-edifice
```

## Ignition libraries are not found

If you see this error message:

```bash
I cannot find any available 'ign' command:
	* Did you install any ignition library?
	* Did you set the IGN_CONFIG_PATH environment variable?
	    E.g.: export IGN_CONFIG_PATH=$HOME/local/share/ignition
```

You should set up the environment variable `IGN_CONFIG_PATH=/usr/local/share/ignition/`
