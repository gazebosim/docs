# Simulate multiple robots with ROS 2 and Gazebo

This tutorial shows how to create multiple robots from the same robot SDF file using namespaces to isolate their topics and services. It also shows how automated bridging can discover and bridge matching ROS 2 and Gazebo interfaces without requiring a separate bridge configuration for each robot.

## Start the demo

Run:

```bash
ros2 launch ros_gz_sim_demos multi_robot.launch.xml
```

The demo uses the [multi_robot.launch.xml](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/multi_robot.launch.xml) launch file to start the simulation with the [multi_robot.sdf](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/worlds/multi_robot.sdf) world. Together, they create four instances of the same vehicle model.

Although all four robots use the same vehicle SDF, each instance has its own namespace. Relative topics and services are resolved within that namespace, so interfaces from different robots remain isolated.

## Isolate robots with namespaces

When multiple instances of the same robot are created in one simulation, they often expose the same relative topic and service names, such as `cmd_vel`, `odom`, or `enable`. Without additional scoping, these interfaces can conflict or make it difficult to address each robot independently.

Namespaces provide that separation without requiring a separate SDF file for every robot. Each instance can use the same vehicle SDF while assigning its relative topics and services to a different namespace. For example, the same `cmd_vel` topic can become `/robot1_ns/cmd_vel` for one robot and `/robot2_ns/cmd_vel` for another.

The demo creates four instances of the same vehicle model using different creation methods, with two defined directly in the world SDF and two spawned from the launch file at runtime.

For example, `robot2` is included from the shared vehicle SDF and assigned a namespace with the `<namespace>` element:

```xml id="m4q2le"
<include>
  <uri>package://ros_gz_sim_demos/models/vehicle</uri>
  <name>robot2</name>
  <namespace>{name}_ns</namespace>
  <pose>0 2 1 0 0 0</pose>
</include>
```

Here, `{name}` resolves to `robot2`, so its relative topics and services are scoped under `/robot2_ns`. The robot can therefore coexist with other instances of the same model without their interfaces conflicting.

The four creation methods used in the demo are:

| Model | Creation method | Namespace setting |
| --- | --- | --- |
| `vehicle` | `<model>` in the world SDF | `namespace="robot1_ns"` |
| `robot2` | `<include>` in the world SDF | `<namespace>{name}_ns</namespace>` |
| `robot3` | `ros_gz_sim create` | `-ns {name}_ns` |
| `robot4` | `gz_spawn_model` | `entity_namespace="'{name}_ns'"` |

All model creation methods support the `{name}` placeholder in namespaces. The `{name}` placeholder can appear multiple times and is resolved to the final name of the corresponding model.

For more detailed examples of creating namespaced models with the supported interfaces, see the [multi_robot demo README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/README.md).

## Bridge robot commands automatically

In a multi-robot simulation, many topics and services have the same purpose and message type, but differ only by their robot namespace. For example, each robot may have its own `/robot1/cmd_vel`, `/robot2/cmd_vel`, and so on. Manually configuring a bridge for every namespaced topic can quickly become repetitive.

Automated bridging avoids this per-robot configuration. It periodically discovers topics and services on both the ROS 2 and Gazebo sides, and creates bridges automatically when endpoints have the same full name and compatible types. As a result, newly spawned robots can also use the same bridging setup without adding new entries to the bridge configuration file.

The demo launch file starts `ros_gz_bridge` with automated bridging enabled:

```xml
<ros_gz_bridge
  bridge_name="ros_gz_bridge"
  config_file="$(find-pkg-share ros_gz_sim_demos)/config/multi_robot.yaml"
  use_composition="True">
  <param name="automated_bridge.enable" value="true" />
  <param name="automated_bridge.exclude_patterns"
         value="['.*camera.*']" />
</ros_gz_bridge>
```

Automated bridging does not need to handle every discovered topic or service. The `automated_bridge.exclude_patterns` parameter can be used to skip topics or services that should be handled separately. For example, image topics may be bridged by another process for performance reasons, or handled with `ros_gz_image` instead.

The `automated_bridge.exclude_patterns` parameter supports regular expression matching. In this demo, `.*camera.*` excludes every discovered topic or service whose name contains `camera`, so these topics and services will be skipped by automated bridging even if corresponding endpoints are discovered on both ROS 2 and Gazebo.

Manually configured bridges are processed first, and automated bridging does not create a duplicate bridge for an interface that is already bridged. 
In this demo, the [multi_robot.yaml](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/config/multi_robot.yaml) configuration file defines one explicit bridge, from Gazebo `/clock` to ROS 2 `/clock`.

## Control the robots from ROS 2

With namespace support, each robot uses its own namespaced topics, so commands sent to one robot do not affect the others. With automated bridging enabled, there is no need to add a bridge configuration for each robot manually. The bridge discovers the matching ROS 2 and Gazebo endpoints automatically.

Open another terminal and publish a command to `robot1`:

```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

After discovery, `robot1` should drive forward while the other robots remain still.

> **NOTE:** Automated bridging requires compatible endpoints with identifiable
> message types to be discovered on both ROS 2 and Gazebo. CLI tools such as
> `ros2 topic echo` and `gz topic -e` may not provide sufficiently specific type
> information when used as endpoints, so they may not trigger automated
> bridging as expected.
