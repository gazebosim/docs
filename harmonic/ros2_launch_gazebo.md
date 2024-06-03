# Launch Gazebo from ROS 2

Gazebo can be launched from a ROS 2 launch system in multiple ways:

## Using the launch files included in
[ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim).

The package `ros_gz_sim` contains two launch files named `gz_server.launch.py`
and `gz_sim.launch.py`. You can use them to start Gazebo server or Gazebo
respectively.

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

Or you can just start the server:

```
ros2 launch ros_gz_sim gz_server.launch.py world_sdf_file:=empty.sdf
```

Consult the argument block of each launch file
[here](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in#L75-L96)
and [here](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_server.launch.py#L27-L38)
to learn about the different parameters that are accepted by each launch file.

## Using a custom launch file.

It's also possible to start Gazebo from your custom launch file. For that
purpose we have created the custom `<gz_server/>` tag that can be used from you
XML or YAML launch file. In this case, the arguments are passed as attributes
within this tag. Here's an example for launching Gazebo server:

```xml
<launch>
  <arg name="world_sdf_file" default="empty.sdf" />
  <arg name="world_sdf_string" default="" />
  <arg name="container_name" default="ros_gz_container" />
  <arg name="use_composition" default="False" />
  <gz_server 
    world_sdf_file="$(var world_sdf_file)"
    world_sdf_string="$(var world_sdf_string)"
    container_name="$(var container_name)"
    use_composition="$(var use_composition)">
  </gz_server>
</launch>
```

In this case the `<gz_server>` parameters are read from the command line. That's
an option but not strictly necessary as you could decide to hardcode some of the
values.
