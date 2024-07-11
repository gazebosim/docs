# ROS Integration

Most of the Ignition libraries use [Ignition Transport](https://github.com/gazebosim/gz-transport)
to exchange data between different software components. This tutorial details
how to interface with [ROS](http://www.ros.org/), enabling the ability to use
tools such as [Rviz](http://wiki.ros.org/rviz) for robot or sensor
visualization.

> **Important**: The packages documented here used to have the `ros1_` prefix
> instead of `ros_`. Please update your code accordingly in order to avoid
> adverse effects.

# ros_ign_bridge to the rescue

ros_ign_bridge provides a network bridge which enables the exchange of messages
between ROS 1 and Ignition Transport. Its support is limited to only certain
message types. Please, read this [README](https://github.com/osrf/ros_ign_bridge)
to verify if your message type is supported by the bridge.

# How to install ros_ign_bridge

## Binary install

The repository `packages.osrfoundation.org` hosts binary packages for Ubuntu Bionic.
To install the binaries

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-get install ros-ign-bridge
```

## Source install

See the [prerequisites](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_bridge#prerequisites)
and [install section](https://github.com/osrf/ros_ign/tree/melodic/ros_ign_bridge#building-the-bridge-from-source)
of the bridge documentation. This tutorial requires Ubuntu Bionic or newer.

Assuming that you have ROS Melodic, you can install all dependencies with:

```bash
sudo apt install ros-melodic-desktop ros-melodic-rqt-image-view libignition-common3-dev libignition-transport7-dev
```

# Run the bridge and exchange images

In this example, we're going to generate Ignition Transport images using Gazebo, that will be converted into ROS 1 images, and visualized with rqt_viewer.

First we start a ROS 1 `roscore`:

```bash
# Shell A:
. /opt/ros/melodic/setup.bash
roscore
```

Then we start Gazebo.

```bash
# Shell B:
ign gazebo -r camera_sensor.sdf
```

Gazebo should be running and publishing images over the `/camera` topic.
Let's verify it:

```bash
# Shell C:
ign topic -l | grep "^/camera"
/camera
```

Then we start the parameter bridge with the previous topic.

If you perform a from source installation, please source the
setup.bash (not needed for binary packages):

```bash
# Shell D:
. ~/bridge_ws/install/setup.bash
```

Run the bridge:

```bash
# Shell D:
rosrun ros_ign_bridge parameter_bridge /camera@sensor_msgs/Image@ignition.msgs.Image
```

Now we start the ROS 1 GUI:

```bash
# Shell E:
. /opt/ros/melodic/setup.bash
rqt_image_view /camera
```

You should see the current images in `rqt_image_view` which are coming from
Gazebo (published as Ignition Msgs over Ignition Transport).

The screenshot shows all the shell windows and their expected content
(it was taken using ROS Melodic):


![Ignition Transport images and ROS 1 rqt](img/bridge_image_exchange_gz-sim.png)
