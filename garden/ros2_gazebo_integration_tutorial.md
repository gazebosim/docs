# ROS 2 and Gazebo Integration Tutorial

`rrbot` is a simple two-link robotic arm with revolute joints, and this tutorial serves as an illustrative example using the same to demonstrate the interoperability of ROS 2 and Gazebo Sim.

## Overview
`ros_gz_project_template` provides an organized structure for ROS 2 and Gazebo projects, including necessary directories, build files, and launch scripts. We'll leverage ROS 2's communication and control mechanisms to interact with the robot. Gazebo is used as the simulation environment, providing realistic physics simulation and visualization of the `rrbot` robot arm. The robot model is described using SDFormat (Simulation Description Format), which defines the robot's structure, joints, links, and their properties. While Gazebo's physics engine simulates the dynamics of the robot, RViz, a visualization tool in ROS 2, displays the robot model and its motion based on data published by the control nodes, all using the same robot description file. Users can interact with `rrbot` through ROS 2 commands to move the robot's arms or retrieve joint state information.


## Prerequisites

1. A working installation of ROS 2 and Gazebo is required to go further. Please follow the [Install Gazebo and ROS document](docs/ros_installation). 
1. Basic familiarity with ROS concepts and terminal commands. 
1. Checkout [ROS 2 Integration](docs/garden/ros2_integration) to get familiar with `ros_gz_bridge` before starting this tutorial.


## Setup

Clone the `ros_gz_project_template` to try the example setup. See [Getting Started with `ros_gz_project_template` for ROS 2 and Gazebo Development](docs/garden/ros_gz_project_template) for guidance on using the template.

// TODO: don't need the template to explain, only reference it for full source code


### Load robot description to the parameter server
```python
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'rrbot', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
```

### Launch state publisher nodes

The `joint_state_publisher` reads the `robot_description` parameter from the parameter server, finds all of the non-fixed joints and publishes a [JointState](https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) message with all those joints defined.

```python
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )
```

The `robot_state_publisher` takes the description and joint angles of the robot as inputs and publishes the 3D poses of the robot links, using a kinematic tree model of the robot.
This is typically helpful when you want robot_state_publisher to publish tf transforms for a robot description, but also want to provide the robot description via code. Take SDFormat XML describing a robot in simulation and publish joint states from the simulation, letting robot_state_publisher handle turning those joint states into tf transforms.

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

// Diagram
![jsp](tutorials/ros2_integration/jsp_diagram.jpg)

### Run RViz
Visualize in RViz and with the help of the GUI, configure your robot model.

// TODO: GIF


## Conclusion

This functionality is useful during initial development of the model. Afterwards, once you've set up simulation, it might be useful while mimicking a joint.
Configure this functionality on your existing ROS and Gazebo project.
 