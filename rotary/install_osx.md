# Binary Installation on MacOS

All the Rotary binaries are available in Ventura and Sonoma using the

[Homebrew package manager](https://brew.sh/). After installing Homebrew, Gazebo Rotary can be installed running:

```bash
brew tap osrf/simulation
brew install gz-rotary
```

All libraries should be ready to use and the `gz sim -s` server app ready to be executed.

Head back to the [Getting started](getstarted)
page to start using Gazebo!

## Uninstalling binary install

If you need to uninstall Gazebo or switch to a source-based install once you
have already installed the library from binaries, run the following command:

```bash
brew uninstall gz-rotary
```

## Troubleshooting

See [Troubleshooting](troubleshooting.md#macos)
