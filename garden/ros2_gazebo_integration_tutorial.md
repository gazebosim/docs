# ROS 2 and Gazebo Integration Tutorial

`rrbot` is a simple two-link robotic arm with revolute joints, and this tutorial serves as an illustrative example to demonstrate the interoperability of ROS 2 and Gazebo Sim.

Overview:
`ros_gz_project_template` provides a standardized structure for ROS 2 and Gazebo projects, including necessary directories, build files, and launch scripts. We'll leverage ROS 2's communication and control mechanisms to interact with the robot. Gazebo is used as the simulation environment, providing realistic physics simulation and visualization of the `rrbot` robot arm. The robot model is described using SDFormat (Simulation Description Format), which defines the robot's structure, joints, links, and their properties. While Gazebo's physics engine simulates the dynamics of the robot, RViz, a visualization tool in ROS 2, displays the robot model and its motion based on data published by the control nodes, all using the same robot description file. Users can interact with `rrbot` through ROS 2 commands to move the robot's arms or retrieve joint state information.
