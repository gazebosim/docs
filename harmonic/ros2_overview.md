# ROS 2 integration overview

Gazebo can be integrated within a ROS 2 system. Let's start describing the
different types of integrations that you can achieve between Gazebo and ROS.

* Use ROS to launch Gazebo: ROS prescribes a specific way to launch all
the pieces needed in your system. There's a dedicated
[launch mechanism](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
to orchestrate the launch of all your components and many tooling around it.
Gazebo can be launched in this way.

* Use ROS to interact with Gazebo topics via the [`ros_gz` bridge](https://github.com/gazebosim/ros_gz):
Once Gazebo is up and running, it's very common to communicate with the
simulation. A common way to perform this communication is via topics. Gazebo has
its own middleware, Gazebo Transport, that exposes a set of topics and services quite similar to ROS. The `ros_gz` bridge allows you to create a bridge between
Gazebo and your ROS system, that translates between Gazebo Transport and ROS 2
as needed.

* Use ROS to spawn a Gazebo model: Gazebo worlds can include models that are
loaded at startup. However, sometimes you need to spawn models at runtime. This
task can be performed using ROS 2.

## Requirements

Please follow the [Install Gazebo and ROS document](ros_installation)
before starting this tutorial. A working installation of ROS 2 and Gazebo is
required to go further.

Important: Most of this functionality is only available in ROS 2 Rolling.
We'll backport it to ROS 2 Jazzy soon.

## Composition

If you inspect the parameters of the launch files mentioned in the next
tutorials, you'll notice that we have included in most cases two parameters
named `use_composition` and `create_own_container`. When the `use_composition`
parameter is set to `True`, the associated ROS node will be loaded within a
ROS container. When this happens, all the nodes within the same ROS container
share the same process and can leverage intraprocess communication.

The parameter `create_own_container` only makes sense when `use_composition` is
set to `True`. This parameter lets you control whether you start a ROS
container for your composable nodes or you defer to an external ROS container.

Our recommendation is to always set the `use_composition` parameter to `True`
and decide if you need to create your own container based on your configuration.
Typically, if you're only dealing with your own launch files you'll probably set
`create_own_container` to `True`. On the other hand, if you're using your launch
files as part of a more complex startup where a ROS container is already
present, you should set `create_own_container` to `False` and, instead, set the
parameter `container_name` to the existing container name.

That way, the communication between Gazebo, the bridge, and other potential
ROS nodes will be intraprocess.

![composition_options](images/composition_options.png)

This figure illustrates the concept of composition. The left diagram captures
the idea of not using composition. All the three example nodes are standalone
nodes, and they can talk via interprocess communication using the bridge.
The center diagram represents the scenario where we can use composition and our
own ROS container between Gazebo and the bridge, but there's an additional
consumer node outside that we cannot control. All communication between Gazebo
and the bridge is intraprocess and interprocess between the external consumer
node and the bridge.
The diagram on the right side is using composition across all nodes but the
launch file doesn't start our own container directly. This setup by itself will
not work until you start an external ROS container (manually or via a separate launch file). In this diagram, the external ROS consumer node starts the
container. We're using the Nav2 logo as an example of external ROS 2 consumer
node.

You can learn more about ROS composition in [this tutorial](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Composition.html).

## What's next?

Here are the next follow-up tutorials that you can check to learn more about
Gazebo/ROS integration.

* [How to launch Gazebo from ROS 2](ros2_launch_gazebo).
* [How to use ROS 2 to interact with Gazebo](ros2_integration).
* [Example of using ROS 2 to load a model and interact with it in Gazebo](ros2_interop).
* [How to spawn a Gazebo model from ROS 2](ros2_spawn_model).
