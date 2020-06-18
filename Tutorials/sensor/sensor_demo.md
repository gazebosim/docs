# Sensors

In this tutorial we will learn how to add sensors to our robot and to our environment.
First we will use contact sensor and make our robot stop when in contact.
second we will make our robot avoid obstacles (avoid collision).
You can find the final world of this tutorial [here](sensor_tut)

## Adding IMU sensor

The inertial measurement unit(IMU) will give us the `orientation` of our robot in quaternion, the `angular_velocity` in the three axis(X, Y, Z). And `linear_acceleration` in the three axis. Let's build upon our `moving_robot.sdf` world. You can download it from [here](moving_robot.sdf). To define the `Imu` in our world add this code under the `<world>` tag.

```xml
<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>
```

Now we can add the `IMU` sensor to our robot as follow:

```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

The sensor is usually added to on of the links of our model, we added it under the `chassis` link. Let's describe the tags. `<always_on>` if true the sensor will always be updated according to the update rate.`<update_rate>` the frequency at which the sensor data is generated. `<visualize>` if true the sensor is visualized in the GUI. `<topic>` name of the topic on which data is published.

### Read data from IMU

To read the data from the `IMU`. Run the world in one terminal.

`ign gazebo sensor_tut.sdf`

In another terminal

`ign topic -e -t /imu`

You should see something similar to this

![Imu_message]()

## Contact sensor

Let's introduce different type of sensors which is the `contact` sensor. We will build a small bump and add the contact sensor to it and if the robot hit the bump the robot will stop. As the `IMU` sensor we should first define it by adding the following code.

```xml
<plugin filename="libignition-gazebo-contact-system.so"
        name="ignition::gazebo::systems::Contact">
</plugin>
```

Then we build the model of our bump and add the sensor to it.

```xml
<model name="bump">
    <pose>3 0 0 0 0 0</pose>
    <static>true</static>
    <link name="body">
        <visual name="v1">
            <geometry>
            <box><size>0.1 10 0.01</size></box>
            </geometry>
        </visual>
        <collision name="c1">
            <geometry>
            <box><size>0.1 10 0.01</size></box>
            </geometry>
        </collision>
        <sensor name='sensor_contact' type='contact'>
            <contact>
                <collision>c1</collision>
            </contact>
        </sensor>
    </link>
    <plugin filename="libignition-gazebo-touchplugin-system.so"
            name="ignition::gazebo::systems::TouchPlugin">
        <target>vehicle_blue</target>
        <namespace>trigger</namespace>
        <time>0.001</time>
        <enabled>true</enabled>
    </plugin>
</model>
```

The definition of the `<sensor>` is very simple, we just define the `name` and the `type` of the sensor. Inside the `collision` we define the collision which is `c1`. To achieve the required functionality(explain) we need to use the `TouchPlugin`. Which takes the name of the `<target>` which will be in contact with our bump in our case `vehicle_blue`. `<namespace>` take the name space of our topic, so when our robot hit the bump it will send a message to `/trigger/touched` topic. Run the world in one terminal and in another terminal listen to the `/trigger/touched` topic.

`ign topic -e -t /trigger/touched`

Drive your robot to the bump when you hit the bump you should see a message.

## Lidar

lidar sensor

message LaserScan
{
  /// \brief Optional header data
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

## TODO

* Use the sensor plot plugin
* Documentation of all [sensors](http://sdformat.org/spec?ver=1.7&elem=sensor)
* Name the world and files with the same
* Use sensor visualization (Mihir project)
* Is the sensors render in the GUI
* change the name of the sdf world, avoid (sensors, sensor_demo)
* guide to proto https://developers.google.com/protocol-buffers/docs/cpptutorial