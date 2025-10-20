# Setting up Gazebo in a Continuous Integration (CI) Pipeline

Setting up a Gazebo environment in a Continuous Integration (CI) pipeline can often be a complex and time-consuming task.
It typically involves installing various dependencies, managing repository keys, and ensuring platform compatibility with the Gazebo distribution.
If done incorrectly, this can lead to unpredictable behaviors in CI due inconsistencies across environments.
This section provides information on setting up a Gazebo in a continuous integration pipeline using the `setup-gazebo` GitHub Action to automate the entire process of configuring a Gazebo environment in a CI by taking care of everything, from handling dependencies to installing the desired Gazebo distribution.

This action can also be used to install Gazebo alongside ROS 2 along with ROS-Gazebo bridge (`ros_gz`). It handles the installation of the correct version of Gazebo and `ros_gz` from all of all the recommended and supported combinations, along with possible combinations through ROS official binaries as ROS Gazebo vendor packages.

The `setup-gazebo` GitHub Action can be found on the [Marketplace](https://github.com/marketplace/actions/setup-gazebo-environment) with its source code hosted at [github.com/gazebo-tooling/setup-gazebo](https://github.com/gazebo-tooling/setup-gazebo). Also check this [community post](https://community.gazebosim.org/t/introducing-the-setup-gazebo-github-action/3132) for more details.

## Supported Gazebo Releases

The action currently supports all the non-EOL Gazebo releases. See [releases](https://gazebosim.org/docs/all/releases/) for more information.

* Fortress
* Harmonic
* Ionic
* Jetty

## Supported Platforms

The `setup-gazebo` GitHub Action works on three platforms (Ubuntu, macOS and Windows) with the usage instructions for each described below,

* [Ubuntu](https://github.com/marketplace/actions/setup-gazebo-environment#ubuntu)
* [macOS](https://github.com/marketplace/actions/setup-gazebo-environment#macos)
* [Windows](https://github.com/marketplace/actions/setup-gazebo-environment#windows)
