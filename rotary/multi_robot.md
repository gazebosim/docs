# Simulate multiple robots with ROS 2 and Gazebo

This tutorial shows how to create multiple robots from the same robot SDF file using namespaces to isolate their topics and services. It also shows how automated bridging can discover and bridge matching ROS 2 and Gazebo interfaces without requiring a separate bridge configuration for each robot.

## Start the demo

Run:

```bash
ros2 launch ros_gz_sim_demos multi_robot.launch.xml
```

The launch file starts the `multi_robot` world and creates four instances of the same vehicle model.

Although all four robots use the same vehicle SDF, each instance has its own namespace. Relative topics and services are resolved within that namespace, so interfaces from different robots remain isolated.

## Define multiple robots in the world SDF

The [demo world](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/worlds/multi_robot.sdf) creates two instances of the same vehicle model.

One way is to wrap the included model in a `<model>` element and assign the namespace to the wrapper:

```xml
<model name="vehicle" namespace="robot1">
  <self_collide>true</self_collide>
  <pose>0 0 1 0 0 0</pose>
  <include merge="true">
    <uri>package://ros_gz_sim_demos/models/vehicle</uri>
  </include>
</model>
```

Another way is to set both the model name and namespace directly in the `<include>` element:

```xml
<include>
  <uri>package://ros_gz_sim_demos/models/vehicle</uri>
  <name>robot2</name>
  <namespace>{name}</namespace>
  <pose>0 2 1 0 0 0</pose>
</include>
```

The `{name}` placeholder resolves to the final model name. In this example, the second robot is named `robot2`, so it also uses the `robot2` namespace.

Both robots use the same vehicle SDF, but their relative topics and services are placed under different namespaces.

## Spawn namespaced robots at runtime

Besides defining robots directly in the world SDF, additional instances can be spawned at runtime from the same robot SDF file. The namespace can be specified when the robot is created, so each new instance keeps its topics and services separate from the others.

### Using `ros_gz_sim create`

The `ros_gz_sim create` executable accepts a namespace through the `-ns` argument.

The `multi_robot.launch.xml` demo uses it to spawn `robot3`:

```xml
<node
  pkg="ros_gz_sim"
  exec="create"
  args="-world multi_robot
        -file $(find-pkg-share ros_gz_sim_demos)/models/vehicle/model.sdf
        -name robot3
        -ns {name}
        -x 0.0
        -y 4.0
        -z 1.0"
  output="screen" />
```

The same interface can be used from the command line to add another robot to the running world:

```bash
VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"

ros2 run ros_gz_sim create \
  -world multi_robot \
  -file "$VEHICLE_SDF" \
  -name robot5 \
  -ns robot5 \
  -x 0.0 \
  -y 8.0 \
  -z 1.0
```

If `-ns` is omitted or set to an empty string, the namespace behavior follows the source SDF. To explicitly disable any namespace defined in the source SDF, set `-ns /`.

### Using `gz_spawn_model`

The `gz_spawn_model` launch action accepts a namespace through `entity_namespace`.

The demo uses it to spawn `robot4`:

```xml
<gz_spawn_model
  world="multi_robot"
  file="$(find-pkg-share ros_gz_sim_demos)/models/vehicle/model.sdf"
  entity_name="robot4"
  entity_namespace="{name}"
  allow_renaming="false"
  x="0.0"
  y="6.0"
  z="1.0"
  yaw="0.0">
</gz_spawn_model>
```

The corresponding `gz_spawn_model.launch.py` launch file can also be invoked directly:

```bash
VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"

ros2 launch ros_gz_sim gz_spawn_model.launch.py \
  world:=multi_robot \
  file:="$VEHICLE_SDF" \
  entity_name:=robot6 \
  entity_namespace:={name} \
  x:=0.0 \
  y:=10.0 \
  z:=1.0
```

If `entity_namespace` is omitted or set to an empty string, the namespace behavior follows the source SDF. Set `entity_namespace:=/` to explicitly disable any namespace defined in the source SDF.

### Using ROS 2 Simulation Interfaces

The ROS 2 Simulation Interfaces provide ROS 2 services for controlling and interacting with the simulation. The `/gzserver/spawn_entity` service can
create another robot instance and assign its namespace through `entity_namespace`:

```bash
VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"

ros2 service call /gzserver/spawn_entity simulation_interfaces/srv/SpawnEntity "{
  name: 'robot7',
  entity_resource: {
    uri: '$VEHICLE_SDF'
  },
  entity_namespace: '{name}',
  allow_renaming: false,
  initial_pose: {
    pose: {
      position: {x: 0.0, y: 12.0, z: 1.0},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    }
  }
}"
```

As with the other interfaces, an empty `entity_namespace` follows the namespace behavior of the source SDF, while `/` explicitly disables it.

### Using Gazebo services

The Gazebo `/world/multi_robot/create/blocking` service can also create a robot from the same SDF file. The model name and namespace are specified directly in the `EntityFactory` request:

```bash
VEHICLE_SDF="$(ros2 pkg prefix --share ros_gz_sim_demos)/models/vehicle/model.sdf"

gz service -s /world/multi_robot/create/blocking \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "'"$VEHICLE_SDF"'",
         name: "robot8",
         entity_namespace: "robot8",
         pose: {
           position: {x: 0.0, y: 14.0, z: 1.0},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
         }'
```

If `entity_namespace` is omitted or empty, the namespace behavior follows the source SDF. Set it to `/` to explicitly disable any namespace defined in the source SDF.

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
In this demo, `config_file` contains one explicit bridge, from Gazebo `/clock` to ROS 2 `/clock`.

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
