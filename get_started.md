# Getting Started with Gazebo?

Welcome to Gazebo!

When you're ready, follow the next few steps to get up and running with
simulation using Gazebo.

## Step 1: Install

<div class="warning">
  <strong>Note:</strong> If you are a <a href="https://ros.org">ROS</a> user, please first read our tutorial about
  the <a href="docs/latest/ros_installation">ROS/Gazebo installation</a>.
</div>

The recommended installation for new users is the use of binary
packages available for the platform to use:

|Platform|Gazebo Versions|
|---|---|
| Ubuntu 22.04 Jammy | [Gazebo Harmonic](/docs/harmonic/install_ubuntu){.external} (recommended), [Gazebo Garden](/docs/garden/install_ubuntu){.external} and [Gazebo Fortress](/docs/fortress/install_ubuntu){.external} (recommended if using ROS 2 Humble or Iron)
| Ubuntu 20.04 Focal | [Gazebo Garden](/docs/garden/install_ubuntu){.external} (recommended), [Gazebo Fortress](/docs/fortress/install_ubuntu){.external} and [Gazebo Citadel](/docs/citadel/install_ubuntu){.external}
| Ubuntu 18.04 Bionic | [Gazebo Citadel](/docs/citadel/install_ubuntu){.external}
| Mac Ventura | [Gazebo Harmonic](/docs/harmonic/install_osx){.external} (recommended), [Gazebo Garden](/docs/garden/install_osx){.external}, [Gazebo Fortress](/docs/fortress/install_osx){.external} and [Gazebo Citadel](/docs/citadel/install_osx){.external}
| Mac Monterey | [Gazebo Harmonic](/docs/harmonic/install_osx){.external} (recommended), [Gazebo Garden](/docs/garden/install_osx){.external}, [Gazebo Fortress](/docs/fortress/install_osx){.external} and [Gazebo Citadel](/docs/citadel/install_osx){.external}
| Windows | Support via Conda-Forge is not fully functional, as there are known runtime issues [see this issue for details](https://github.com/gazebosim/gz-sim/issues/168).

If the desired platform is not listed above or if a particular feature in a
given [Gazebo release](releases) is needed,
there is an installation package per release available with all the
installation options:

* [Gazebo Harmonic installation](/docs/harmonic/install){.external} options (EOL 2028 Sep)
* [Gazebo Garden installation](/docs/garden/install){.external} options (EOL 2024 Sep)
* [Gazebo Fortress (LTS) installation](/docs/fortress/install){.external} options (EOL 2026 Sep)
* [Gazebo Citadel (LTS) installation](/docs/citadel/install){.external} options (EOL 2024 Dec)

## Step 2: Run

After installing Gazebo in Step 1, you can launch Gazebo Sim, a 3D robotics
simulator, from a terminal.

* If you are on macOS, see specific instructions in the [macOS section](#macos).

Launch Gazebo by running:

```
gz sim shapes.sdf  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```

This command will launch both the Sim server and Sim GUI with a world
that contains three simple shapes.

Add the `-v 4` command line argument to generate error, warning,
informational, and debugging messages on the console.

```
gz sim shapes.sdf -v 4  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```

Gazebo Sim can also be run headless, i.e. without the GUI, by using the `-s` (server only) flag.

```
gz sim -s shapes.sdf -v 4  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
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
Starting with Citadel, there are more [versioned tutorials](/docs/citadel/tutorials){.external}
covering the basics of the GUI, creating worlds and robots, and more.

Each [Gazebo library](/libs) also has a set of tutorials and
examples. Explore these resources, and don't forget to ask questions and
find solutions at [answers.gazebosim.org](http://answers.gazebosim.org).

## macOS

On macOS, you will need to run Gazebo using two terminals, one for the server
and another for the GUI:

```sh
# launch server in one terminal
gz sim -v 4 shapes.sdf -s  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```

```sh
# launch gui in a separate terminal
gz sim -v 4 -g  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```

The GUI on macOS is currently known to be unstable. Basic interaction with
the 3D scene such as camera view control and translation / rotation tools
should be functional. However, some GUI plugins like the Component Inspector
may be buggy and interaction with certain GUI elements may cause the GUI
to crash. Please ticket an issue at https://github.com/gazebosim/gz-sim/
if you run into GUI problems on macOS.
