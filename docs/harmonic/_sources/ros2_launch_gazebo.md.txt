# Launch Gazebo from ROS 2

Gazebo can be launched from a ROS 2 launch system in multiple ways:

## Using the launch files included in
[ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim).

The package `ros_gz_sim` contains two launch files named `gz_server.launch.py`
and `gz_sim.launch.py`. You can use them to start Gazebo server or Gazebo (server and GUI)
respectively.

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

Or you can just start the server:

```bash
ros2 launch ros_gz_sim gz_server.launch.py world_sdf_file:=empty.sdf
```

Consult the argument block of each launch file
[here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/gz_sim.launch.py.in#L75-L96)
and [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/gz_server.launch.py#L27-L38)
to learn about the different parameters that are accepted by each launch file.

## Using a custom launch file.

### XML
It's also possible to start Gazebo from your custom launch file. For that
purpose we have created the custom `<gz_server/>` tag that can be used from your
XML launch file. In this case, the arguments are passed as attributes
within this tag. Here's an example for launching Gazebo server:

```xml
<launch>
  <arg name="world_sdf_file" default="empty.sdf" />
  <arg name="world_sdf_string" default="" />
  <arg name="container_name" default="ros_gz_container" />
  <arg name="create_own_container" default="False" />
  <arg name="use_composition" default="False" />
  <gz_server 
    world_sdf_file="$(var world_sdf_file)"
    world_sdf_string="$(var world_sdf_string)"
    container_name="$(var container_name)"
    create_own_container="$(var create_own_container)"
    use_composition="$(var use_composition)">
  </gz_server>
</launch>
```

In this case the `<gz_server>` parameters are read from the command line. That's
an option but not strictly necessary as you could decide to hardcode some of the
values or not even use all the parameters.

### Python
Python launch files provide more low-level customization and logic compared to XML launch files. 
In the following example, the user can specify a world argument to launch an environment for
the Moon, Mars, or Enceladus. It additionally sets the resource path environment variable and
sets up empty arrays for topics to be bridged and remapped from Gazebo to ROS 2:
```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='moon',
            choices=['moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])
```

Here's another example using a higher level action from Python to launch `gzserver`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_sim.actions import GzServer


def generate_launch_description():

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value='',
        description='Path to the SDF world file')

    # Create the launch description and populate
    ld = LaunchDescription([
        GzServer(
            world_sdf_file=LaunchConfiguration('world_sdf_file')
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_world_sdf_file_cmd)

    return ld
```


## Launching with ros_gz_bridge

An example launch file for XML can be viewed [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_sim.launch)
An example launch file for Python can be viewed [here](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/launch/ros_gz_sim.launch.py)

Example command for directly using these launch files from the terminal:
```bash
ros2 launch ros_gz_sim ros_gz_sim.launch.py world_sdf_file:=empty.sdf bridge_name:=ros_gz_bridge config_file:=<path_to_your_YAML_file> use_composition:=True create_own_container:=True
```

In the above launch files you may notice that the `create_own_container` argument for `ros_gz_bridge` is hardcoded to `False`. This has been done to prevent two duplicate containers from getting created (one for `gz_server` and another one for `ros_gz_bridge`), and instead make `ros_gz_bridge` use the container created by `gz_server`. More info about this can be viewed [here](https://github.com/gazebosim/ros_gz/pull/620#issue-2595570189)

More info about `ros_gz_bridge` can be viewed [here](ros2_integration).
More info about composition can be viewed [here](ros2_overview#composition).
