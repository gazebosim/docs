# ROS 2 integration overview

Gazebo can be integrated within a ROS 2 system. Let's start describing the
different types of integrations that you can achieve between Gazebo and ROS.

* Use ROS to launch Gazebo: ROS prescribes a specific way to launch all
the pieces needed in your system. There's a dedicated
[launch mechanism](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
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

Important: Most of this functionality is only available in ROS2 Rolling.
We'll backport it to ROS 2 Jazzy soon.

## Composition

If you inspect the parameters of the launch files mentioned in the next
tutorials, you'll notice that we have included in most cases a parameter named
`use_composition`. When that parameter is set to `True`, the associated ROS
node will be included within a ROS container. When this happens all the nodes
live within the same process and can leverage intraprocess communication.

Our recommendation is to always set the `use_composition` parameter to `True`.
That way, the communication between Gazebo and the bridge will be intraprocess.
If your ROS nodes are also written as composable nodes, make sure that they are
launched with the `container_node_name` parameter matching the container name
including Gazebo and the bridge.

You can learn more about ROS composition in [this tutorial](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Composition.html).

## What's next?

Here are the next follow-up tutorials that you can check to learn more about
Gazebo/ROS integration.

* [How to launch Gazebo from ROS 2](ros2_launch_gazebo).
* [How to use ROS 2 to interact with Gazebo](ros2_integration).
* [Example of using ROS 2 to load a model and interact with it in Gazebo](ros2_interop).
* [How to spawn a Gazebo model from ROS 2](ros2_spawn_model).
