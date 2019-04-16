# Getting started with Ignition Robotics?

Welcome to Ignition Robotics!

If you're here and not quite sure what Ignition Robotics is all about, then
you might want to give the [Overview](/docs/all/overview) page a quick read.

When you're ready, follow the next few steps to get up and running with
simulation using Ignition Robotics.

## Step 1: Install

Each release of Ignition ships with a new installation tutorial. You can
read the latest installation tutorial at
[ignitionrobotics.org/docs/latest/install](/docs/latest/install). Please
visit the [main documentation](/docs) pages for a list of all releases,
along with links to their respective installation tutorials.

## Step 2: Run

After installing Ignition in Step 1, you can launch Gazebo, a 3D robotics
simulator, from a terminal using

```
ign-gazebo -f shapes.sdf
```

This command will launch both the Gazebo server and Gazebo GUI with a world
that contains three simple shapes.

Add the `-v 4` command line argument to generate error, warning,
informational, and debugging messages on the console.

```
ign-gazebo -f shapes.sdf -v 4
```

Gazebo can also be run headless, i.e. without the GUI, by using `ign-gazebo-server` instead of `ign-gazebo`. For example,

```
ign-gazebo-server -f shapes.sdf -v 4
```

Similarly, the GUI can be run independently using `ign-gazebo-gui`. On
start, the GUI will attempt to connect to an `ign-gazebo-server` instance.
If a server is not available, then you will see just a blank screen until
a server instances is started.

## Step 3: Create your own world

[SDF](http://sdformat.org/) is used to specify the contents of simulation.
Take a look at the available [SDF tutorials](http://sdformat.org/tutorials)
to get started.

Modifying an existing SDF world is also a good way to get started. Ignition
Gazebo ships with a number of [example SDF
worlds](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/examples/worlds)
that you can freely copy and modify.  

There are a wide variety of simulation resources at your disposal on
[https://app.ignitionrobotics.org/fuel](https://app.ignitionrobotics.org/fuel).
If you find a model you'd like to use, then click on the `<>` icon in the
model description page, highlighted in the image below, to copy an SDF
snippet into your clipboard. This snippet can be pasted directly into your
custom SDF file.

![SDF model snippet](images/model_snippet.png)


## Step 4: Explore and learn

This tutorial has covered the basics of getting started with Ignition
Robotics. Each [Ignition library](/libs) has a set of tutorials and
examples. Explore these resources, and don't forget to ask questions and
find solutions at [answers.gazebosim.org](http://answers.gazebosim.org).
