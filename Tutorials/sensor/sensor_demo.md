# Sensors

In this tutorial we will learn how to add sensors to our robot and to our environment. we will use three different sensors the IMU sensor, Contact sensor and Lidar sensor.
You can find the final world of this tutorial [here](sensor_tut)

## Adding IMU sensor

The inertial measurement unit(IMU) will give us the `orientation` of our robot in quaternion, the `angular_velocity` in the three axis(X, Y, Z). And `linear_acceleration` in the three axis. Let's build upon our `moving_robot.sdf` world. You can download it from [here](moving_robot.sdf). To define the `IMU` in our world add this code under the `<world>` tag.

```xml
<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>
```

This code defines the `IMU` sensor to be used in our world. now we can add the `IMU` sensor to our robot as follow:

```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

The sensor is usually added to one of the links of our model, we added it under the `chassis` link. Let's describe the tags. `<always_on>` if true the sensor will always be updated according to the update rate.`<update_rate>` the frequency at which the sensor data is generated. `<visualize>` if true the sensor is visualized in the GUI. `<topic>` name of the topic on which data is published.

### Read data from IMU

To read the data from the `IMU`. Run the world in one terminal.

`ign gazebo sensor_tut.sdf`

In another terminal

`ign topic -e -t /imu`

Move your robot around using arrow keys on the keyboard and you should see something similar to this which are the `orientation`, `angular_velocity` and `linear_acceleration` as described above.

![Imu_message]()

## Contact sensor

Let's introduce different type of sensors which is the `contact` sensor. You can guess from the name this sensor give indication when it touch(in contact) with something else. We will build a wall and add the contact sensor to it and if the robot hit the wall the robot will stop to prevent it from damaging itself. Let's first build the wall as follow.

```xml
<model name='wall'>
    <pose>5 0 0 0 0 0</pose><!--pose relative to the world-->
    <link name='box'>
        <pose/>
        <inertial> <!--inertial properties of the link mass, inertia matrix-->
            <mass>1.14395</mass>
            <inertia>
                <ixx>0.126164</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.416519</iyy>
                <iyz>0</iyz>
                <izz>0.481014</izz>
            </inertia>
        </inertial>
        <visual name='visual'>
            <geometry>
                <box>
                    <size>0.5 10.0 2.0</size>
                </box>
            </geometry>
            <!--let's add color to our link-->
            <material>
                <ambient>0.0 0.0 1.0 1</ambient>
                <diffuse>0.0 0.0 1.0 1</diffuse>
                <specular>0.0 0.0 1.0 1</specular>
            </material>
        </visual>
        <collision name='collision'>
            <geometry>
                <box>
                    <size>0.5 10.0 2.0</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

You can check the SDF [tutorial]() to know how to build your own world. Now run the world and make sure that the wall appear in the simulation like this

![wall_in_world]()

Let's add the contact sensor to the wall. As the `IMU` sensor we should first define `Contact` sensor by adding the following code.

```xml
<plugin filename="libignition-gazebo-contact-system.so"
        name="ignition::gazebo::systems::Contact">
</plugin>
```

Now we can add the `contact` sensor to `box` link of `wall` model.

```xml
<sensor name='sensor_contact' type='contact'>
    <contact>
        <collision>collision</collision>
    </contact>
</sensor>

The definition of the `<sensor>` is very simple, we just define the `name` and the `type` of the sensor. Inside the `collision` we define the collision name which is `collision`. We need also to use the `TouchPlugin` as follow:  

```xml
<plugin filename="libignition-gazebo-touchplugin-system.so"
        name="ignition::gazebo::systems::TouchPlugin">
    <target>vehicle_blue</target>
    <namespace>wall</namespace>
    <time>0.001</time>
    <enabled>true</enabled>
</plugin>
```

 The `TouchPlugin` will publish(send) message when the `wall` been touched. The tags of the plugin are as follow. `<target>` which will be in contact with our wall in our case `vehicle_blue`. `<namespace>` take the name space of our topic, so when our robot hit the wall it will send a message to `/wall/touched` topic. Run the world in one terminal and in another terminal listen to the `/trigger/touched` topic.

`ign topic -e -t /trigger/touched`

Drive your robot to the wall when you hit the bump you should see a message `data: true`. Now we can use the `TriggeredPublisher` plugin to make our robot stop when hit the wall as follows:

```xml
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Boolean" topic="/wall/touched">
        <match>data: true</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.0}, angular: {z: 0.0}
    </output>
</plugin>
```

As explained in the Moving robot [tutorial](moving robot.md), we can publish output depend on a received input. So here we publish `linear: {x: 0.0}, angular: {z: 0.0}` to make our robot stop.

## Lidar sensor

We don't want our robot to touch the wall at all because this may cause some damage, so instead of contact sensor we can use the Lidar. Lidar is an acronym for light detection and ranging. This sensor can help us to detect obstacles around our robot. We will use it to measure the distance between our robot and the wall.

First let's create a frame on the `chassis` to fix our lidar to it.

```xml
<frame name="lidar_frame" attached_to='chassis'>
    <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```

Then add this plugin under the `<world>` tag, to be able to use the `lidar` sensor.

```xml
    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
```

Under the `chassis` frame we can add the `lidar` sensor as follow:

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>"
    <pose relative_to='caster_frame'>0 0 0.7 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <alwaysOn>1</alwaysOn>
    <visualize>true</visualize>
</sensor>
```

First we defined the `name` and `type` of our sensor, then we defined its `<pose>` relative to the `chassis`. In `<Topic>` we define the topic on which the lidar data will be published. `<update_rate>` is the frequency at which the `lidar` data is generated, in our case `10 Hz` which is equal to `0.1 Sec`. Under the `<horizontal>` and `<vertical>` tags we define the properties of the horizontal and vertical laser rays. `<samples>` is the number of simulated lidar rays to generate per complete laser sweep cycle. `<resolution>` this number is multiplied by samples to determine the number range data points. The `<min_angle>` and `<max_angle>` are the angle range of the generated rays. Under the `<range>` we define range properties of each simulated ray. `<min>` and `<max>` define the minimum and maximum distance for each lidar ray. The `<resolution>` tag here define the linear resolution of each lidar ray. `<alwaysOn>` if true the sensor will always be updated according to the `<update_rate>`. `<visualize>` if true the sensor is visualized in the GUI.

Now run the world and look at the lidar messages on the `/lidar` topic as you drive the robot around, specifically the `ranges` data.

`ign gazebo sensor_tut.sdf`

`ign topic -e -t /lidar`

The lidar message has following attributes.

message LaserScan
{
  Header header              = 1;

  string frame               = 2;
  Pose world_pose            = 3;
  double angle_min           = 4;
  double angle_max           = 5;
  double angle_step          = 6;
  double range_min           = 7;
  double range_max           = 8;
  uint32 count               = 9;
  double vertical_angle_min  = 10;
  double vertical_angle_max  = 11;
  double vertical_angle_step = 12;
  uint32 vertical_count      = 13;

  repeated double ranges              = 14;
  repeated double intensities         = 15;
}

### Avoid the wall

Now as we have the lidar on our robot, we can use the `ranges` data and make our robot avoid hitting the wall. First we will build a node which subscribe to the `/lidar` topic and read its data. Have a look at this [tutorial](https://ignitionrobotics.org/api/transport/9.0/messages.html) to learn how to build a `publisher` and a `subscriber`. You can download the finished node for this demo from [here](lidar_node.cc)

#### lidar_node

```cpp
ignition::transport::Node node;
std::string topic_pub = "/cmd_vel";
ignition::msgs::Twist data;
auto pub = node.Advertise<ignition::msgs::Twist>(topic_pub);
```

We declare a `node` which will publish to `cmd_vel` topic and define the message type `Twist`.then advertise our node.

```cpp    // Zzzzzz.
    ignition::transport::waitForShutdown();

    return 0;
void cb(const ignition::msgs::LaserScan &_msg)
{
  bool all_more = true;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < 1.0) //if all bigger than one
    {
      all_more = false;
      break;
    }
  }
  if (all_more)
  {
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
  }
  else
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.5);
  }
  pub.Publish(data);
}
```

Inside the callback function we check if range of all the rays are more `1.0` if so we publish message to our car to move forward otherwise we make the robot rotate.

```cpp
int main(int argc, char **argv)
{
    std::string topic = "/lidar";
    // Subscribe to a topic by registering a callback.
    if (!node.Subscribe(topic, cb))
    {
        std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
        return -1;
    }

    // Zzzzzz.
    ignition::transport::waitForShutdown();

    return 0;
}
```

Inside the main we subscribe to the `lidar` topic, and wait till the node is been shutdown.

#### Build the node

Download the [CMakeLists.txt](docs/Tutorials/sensor/CMakeLists.txt) and in the same folder of `lidar_node` create `build/` directory.

```{.sh}
mkdir build
cd build
```

Run cmake and build the code.

```{.sh}
cmake ..
make lidar_node
```

##### Run the node

Run the node from terminal 1:

```{.sh}
./lidar_node
```

Run the world from terminal 2:

`ign gazebo sensor_tut.sdf`

Now you can see the robot move forward and as it approaches the wall it start to turn left till it clear and move forward again.

## TODO

* Use the sensor plot plugin
* Documentation of all [sensors](http://sdformat.org/spec?ver=1.7&elem=sensor)
* Name the world and files with the same
* Use sensor visualization (Mihir project)
* Is the sensors render in the GUI
* change the name of the sdf world, avoid (sensors, sensor_demo)
* guide to proto https://developers.google.com/protocol-buffers/docs/cpptutorial
* inertial properties of the laser_link
* change the color of the wall
* Make all the tags with the same style
* check all the lidar rays not all 10
* link for the sdf tutorial
* screenshots
* explain "ignition::gazebo::systems::Sensors" plugin
* clear the cmakelist file 

## Questions

* question I think lidar without gpu is not supported yet
* Visualize does it mean the rays for example, or the physical sensor
