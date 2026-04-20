# Spawn a Gazebo model from ROS 2

Gazebo will spawn all the models included in the provided world file at startup.
Additionally, it's possible to spawn new models at any time. To do so using ROS
we have provided the following mechanisms:

## Spawn a model using the launch file included in `ros_gz_sim`.

The package `ros_gz_sim` contains a launch file named
`gz_spawn_model.launch.py`. You can use it to spawn a new model into an
existing simulation. Here's an example:

```bash
ros2 launch ros_gz_sim gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5
```

Check [this block](https://github.com/gazebosim/ros_gz/blob/cadae1c8323a74395c09a37e3de4c669c8c09d4f/ros_gz_sim/launch/ros_gz_spawn_model.launch.py#L33-L44)
from the source code to know all the different parameters accepted by this
launch file.

## Spawn a model from a custom launch file.

It's also possible to spawn the model from your custom launch file. For that
purpose we have created the `<gz_spawn_model/>` tag that can be used from you
XML or YAML launch file. In this case, the arguments are passed as attributes
within this tag. Here's an example:

```xml
<launch>
  <arg name="world" default="" />
  <arg name="file" default="" />
  <arg name="model_string" default="" />
  <arg name="topic" default="" />
  <arg name="entity_name" default="" />
  <arg name="allow_renaming" default="False" />
  <arg name="x" default="" />
  <arg name="y" default="" />
  <arg name="z" default="" />
  <arg name="roll" default="" />
  <arg name="pitch" default="" />
  <arg name="yaw" default="" />
  <gz_spawn_model 
    world="$(var world)"
    file="$(var file)"
    model_string="$(var model_string)"
    topic="$(var topic)"
    entity_name="$(var entity_name)"
    allow_renaming="$(var allow_renaming)"
    x="$(var x)"
    y="$(var y)"
    z="$(var z)"
    roll="$(var roll)"
    pitch="$(var pitch)"
    yaw="$(var yaw)">
  </gz_spawn_model>
</launch>
```

In this case the `<gz_spawn_model>` parameters are read from the command line.
That's an option but not strictly necessary as you could decide to hardcode some
of the values or not even use all of the parameters.


## Spawning a model alongside launching ros_gz_bridge

An example launch file for XML can be viewed [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_spawn_model.launch)
An example launch file for Python can be viewed [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_spawn_model.launch.py)

Example command for directly using these launch files from the terminal:
```bash
ros2 launch ros_gz_sim ros_gz_spawn_model.launch.py world:=empty file:=$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf entity_name:=my_vehicle x:=5.0 y:=5.0 z:=0.5 bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file>
```

More info about `ros_gz_bridge` can be viewed [here](ros2_integration).
