# Getting Started with Gazebo?

Welcome to Gazebo!

When you're ready, follow the next few steps to get up and running with
simulation using Gazebo.

## Step 1: Install

***Note:*** If you are a [ROS](ros.org) user, please first read our tutorial about
the [ROS/Gz installation](./ros_installation.md).

The recommended installation for non expert users is the use of binary
packages available for the platform to use when possible.

|Platform|Gz Versions|
|---|---|
| Ubuntu 22.04 Jammy | [Gz Garden](docs/garden/install_ubuntu) (recommended) and [Gz Fortress](docs/fortress/install_ubuntu)
| Ubuntu 20.04 Focal | [Gz Garden](docs/garden/install_buuntu) (recommended), [Gz Fortress](docs/fortress/install_ubuntu) and [Gz Citadel](docs/citadel/install_ubuntu)
| Ubuntu 18.04 Bionic | [Gz Citadel](docs/citadel/install_ubuntu)
| Mac Monterey | [Gz Garden](docs/garden/install_osx) (recommended), [Gz Fortress](docs/fortress/install_osx) and [Gz Citadel](docs/citadel/install_osx)
| Mac BigSur | [Gz Garden](docs/garden/install_osx) (recommended), [Gz Fortress](docs/fortress/install_osx) and [Gz Citadel](docs/citadel/install_osx)
| Mac Catalina | [Gz Garden](docs/garden/install_osx) (recommended), [Gz Fortress](docs/fortress/install_osx) and [Gz Citadel](docs/citadel/install_osx)

Windows support via conda-forge is not fully functional, there are known runtime problems
[stoping Gz to work](https://github.com/gazebosim/gz-sim/issues/168).

If the desired plaform is not listed above or if a particular feature in a
given Gz release is needed, there is a installation package per release
available with all the installation options:

* [Gz Garden](docs/garden/install) installation options
* [Gz Fortress](docs/fortress/install) installation options
* [Gz Citadel](docs/citadel/install) installation options

## Step 2: Run

After installing Gazebo in Step 1, you can launch Gazebo Sim, a 3D robotics
simulator, from a terminal using

```
gz sim shapes.sdf
```

This command will launch both the Sim server and Sim GUI with a world
that contains three simple shapes.

Add the `-v 4` command line argument to generate error, warning,
informational, and debugging messages on the console.

```
gz sim shapes.sdf -v 4
```

Gazebo Sim can also be run headless, i.e. without the GUI, by using the `-s` (server only) flag.

```
gz sim -s shapes.sdf -v 4
```

Similarly, the GUI can be run independently using the `-g` (gui only) flag.
On start, the GUI will attempt to connect to a server instance.
If a server is not available, then you will see just a blank screen until
a server instances is started.

## Step 3: Create your own world

[SDF](http://sdformat.org/) is used to specify the contents of simulation.
Take a look at the available [SDF tutorials](http://sdformat.org/tutorials)
to get started.

Modifying an existing SDF world is also a good way to get started. Gazebo
Sim ships with a number of [example SDF
worlds](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds)
that you can freely copy and modify. These example SDF files are
installed. Many of the SDF files also have instructions located at the
top of the SDF file. The instructions typically contain information about how to
run Sim with the SDF file in order to experience a particular feature.

There are a wide variety of simulation resources at your disposal on
[https://app.gazebosim.org/fuel](https://app.gazebosim.org/fuel).
If you find a model you'd like to use, then click on the `<>` icon in the
model description page, highlighted in the image below, to copy an SDF
snippet into your clipboard. This snippet can be pasted directly into your
custom SDF file.

![SDF model snippet](images/model_snippet.png)


## Step 4: Explore and learn

This tutorial has covered the basics of getting started with Gazebo.
Starting with Citadel, there are more [versioned tutorials](/docs/citadel/tutorials)
covering the basics of the GUI, creating worlds and robots, and more.

Each [Gazebo library](/libs) also has a set of tutorials and
examples. Explore these resources, and don't forget to ask questions and
find solutions at [answers.gazebosim.org](http://answers.gazebosim.org).
