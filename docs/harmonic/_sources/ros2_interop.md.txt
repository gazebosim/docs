# ROS 2 Interoperability

`rrbot` is a simple two-link robotic arm with revolute joints, and this tutorial serves as an illustrative example using the same to demonstrate the interoperability of ROS 2 and Gazebo Sim.

## Overview
We'll leverage ROS 2's communication and control mechanisms to interact with the robot. Gazebo is used as the simulation environment, providing realistic physics simulation and visualization of the `rrbot` robot arm.
The robot model is described using [SDFormat](http://sdformat.org/) (Simulation Description Format), which defines the robot's structure, joints, links, and their properties.
While Gazebo's physics engine simulates the dynamics of the robot, [RViz](https://github.com/ros2/rviz), a visualization tool in ROS 2, displays the robot model and its motion based on data published by the control nodes, all using the same robot description file.
Users can interact with `rrbot` through ROS 2 commands to move the robot's arms or retrieve joint state information.

## Prerequisites

1. A working installation of ROS 2 and Gazebo is required to go further. Please follow the [Install Gazebo and ROS document](/docs/latest/ros_installation). 
2. Basic familiarity with ROS concepts and terminal commands.
3. Check out [ROS 2 Integration](ros2_integration) to get familiar with [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz) before starting this tutorial.

## Setup

Start a fresh ROS 2 Python launch file or add the following nodes in your project's main launch file. We aim to achieve:
1. A robot model development and test setup
2. Configure RViz (and other ROS 2 tools) to control a robot model simulated by a Gazebo world

Note: The full source code for this tutorial can be found in the [ros_gz_example_bringup package](https://github.com/gazebosim/ros_gz_project_template/tree/main/ros_gz_example_bringup/launch) launch files.

## Implementation

### Load robot description to the parameter server

Load the robot description file to set the `robot_description` parameter on the parameter server.

```python
    sdf_file = os.path.join(pkg_project_description, 'models', 'rrbot', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
```

### Launch state publisher nodes

For publishing and controlling the robot pose, we need joint states of the robot.

![jsp](tutorials/ros2_integration/jsp_diagram.jpg)

1. The [`joint_state_publisher`](https://github.com/ros/joint_state_publisher) reads the `robot_description` parameter from the parameter server, finds all of the non-fixed joints and publishes a [JointState](https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) message with all those joints defined.
   ```python
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )
    ```
2. Visualize in RViz and with the help of the `joint_state_publisher_gui`, configure your robot model by adjusting joint states and poses using the slider.

   ![jsp_gui](tutorials/ros2_integration/jsp_gui.png)

   See [documentation](http://docs.ros.org/en/rolling/p/joint_state_publisher_gui/) for node API.
   This functionality is useful during initial development of the model.
   At this point we have achieved the first aim defined in [Setup](#setup). 

3. Now if you'd want to extend this to visualize robot motion, we need positions and transforms.
   The [`robot_state_publisher`](https://github.com/ros/robot_state_publisher) takes the description and joint angles of the robot as inputs and publishes the 3D poses of the robot links, using a kinematic tree model of the robot.
    ```python
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    ```
   These 3D poses are published on the `/tf` topic which is useful to track/plan the robot's motion, communicate between different parts of the robot or evaluate the robot's performance.

### Configure a communication bridge

These joint states can either come from `joint_state_publisher` as seen earlier or from simulated [JointStatePub](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html) system's `joint_state` message.

Configure a bridge between ROS topic `/joint_states` and Gazebo topic `/world/demo/model/diff_drive/joint_state` by adding remappings in the node setup or by creating a [bridge.yaml](https://github.com/gazebosim/ros_gz_project_template/blob/main/ros_gz_example_bringup/config/ros_gz_example_bridge.yaml):

```bash
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/demo/model/diff_drive/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```

Learn more about the bridge from [ROS 2 Integration](ros2_integration).

### Maintaining a single robot description format

The main pain point of using existing simulation assets with ROS 2 tools was updating URDF files into a Gazebo-readable format. This is no longer required. If you are maintaining a URDF and an SDF file in a project, you can now drop the URDF and just use the SDF for both ROS and Gazebo.

This is made possible by `sdformat_urdf`, a parser plugin library that converts an SDF file to URDF C++ DOM structures, making it understandle by the ROS 2 ecosystem.
Although, there are some limitations of the plugin, like not all SDFormat tags are compatible. For example, if you have any sensors attached to a joint, it won't be parsed. More details [here](https://github.com/ros/sdformat_urdf/tree/ros2/sdformat_urdf).

To embed this functionality, we simply need to print the SDFormat file to the `/robot_description` ROS topic, and internally ROS will find a suitable parser, `sdformat_urdf` in this case, to read the file. This is already done while configuring the `robot_state_publisher` earlier.

### Run RViz and Gazebo

The second aim defined in [Setup](#setup) is essentially maintaining only one robot description format which now can be controlled directly with Gazebo.
Using the SDFormat XML description of a robot, the simulator publishes model joint states.
And `robot_state_publisher` handles turning those joint states into `tf`s which is used by RViz and other ROS tools.
This enables visualizing a model in RViz simulated by Gazebo.

![gz_rviz](tutorials/ros2_integration/gz_rviz.gif)

## Conclusion

Configure this functionality to enhance your existing ROS and Gazebo project.
`ros_gz_project_template` provides an organized structure for ROS 2 and Gazebo projects, including necessary directories, build files, and launch scripts.
See [Getting Started with `ros_gz_project_template` for ROS 2 and Gazebo Development](ros_gz_project_template_guide) for guidance on using the template.
