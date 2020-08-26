# Moving the robot

In this tutorial we will learn how to move our robot. We will use the
robot we built in the [Build your own robot](building_robot)
tutorial. You can download the robot from [here](https://github.com/ignitionrobotics/docs/blob/master/tutorials/building_robot/building_robot.sdf).
You can also find the finished world of this tutorial [here](https://github.com/ignitionrobotics/docs/blob/master/tutorials/moving_robot/moving_robot.sdf).

## What is a plugin

To make our robot move we will use the `diff_drive` plugin. But before doing so let's answer the question "What is a plugin?" A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. Plugins make us control many aspects of the simulation like world, models etc.

### Diff_drive plugin

`diff_drive` plugin helps us control our robot, specifically a robot that
can be differentially driven. Let's setup the plugin on our robot. Open
the `building_robot.sdf` and add the following code within the `vehicle_blue`
model tags.

```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

The `<plugin>` tag has two attributes, `filename` which takes the library file name and `name` which takes the name of the plugin.
In the `<left_joint>` and `<right_joint>` tags we define the joints which connect the left and the right wheel with the body of the robot, in our case `left_wheel_joint` and `right_wheel_joint`. `<wheel_separation>` takes the distance between the two wheels.
Our robot has its `left_wheel` at 0.6 m and the `right_wheel` at -0.6 m in y-axis with respect to the `chassis`, so the `wheel_separation` is 1.2 m.
`<wheel_radius>` takes the radius of the wheel which was defined in the `<radius>` tag under the wheel link.
`<odom_publish_frequency>` sets the frequency at which the odometry is published at `/model/vehicle_blue/odometry`.
`cmd_vel` is the input `<topic>` to the `DiffDrive` plugin.

## Topics and Messages

Now our model is ready. We just need to send commands (messages) to it.
These messages will be published (sent) on the `cmd_vel` topic defined above.

A topic is simply a name for grouping a specific set of messages or a particular service.
Our model will subscribe (listen) to the messages sent on the `cmd_vel` topic.

Launch the robot world:

`ign gazebo building_robot.sdf`

In another terminal let's send a message to to our robot:

`ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`

Now you should have your robot moving in the simulation.

**Note:** Don't forget to press the play button in the simulation.

The command specifies the topic to publish to after the `-t` option.
After the `-m` we specify the message type.
Our robot expects messages of type `Twist` which consists of two components, `linear` and `angular`.
After the `-p` option we specify the content (value) of the message: linear speed `x: 0.5` and angular speed `z: 0.05`.

**Hint:** You can know what every topic option does using this command: `ign topic -h`

For more information about `Topics` and `Messages` in Ignition check the [Transport library tutorials](https://ignitionrobotics.org/api/transport/9.0/tutorials.html)

## Moving the robot using the keyboard

Instead of sending messages from the terminal we will send messages using the keyboard keys. To do so we will add two new plugins: `KeyPublisher` and `TriggeredPublisher`.

### KeyPublisher

`KeyPublisher` is an `ign-gui` plugin that reads the keyboard's keystrokes and sends them on a default topic `/keyboard/keypress`.
To use this plugin add the following code under the `<gui>` tag.

```xml
<!-- KeyPublisher plugin-->
<plugin filename="KeyPublisher" name="Key Publisher"/>
```

Let's try this plugin as follows:

* In one terminal type

    `ign gazebo building_robot.sdf`

* In another terminal type

    `ign topic -e -t /keyboard/keypress`

The last command will display all messages sent on `/keyboard/keypress` topic.

In the ignition window press different keys and you should see data(numbers) on the terminal where you run the `ign topic -e -t /keyboard/keypress` command.

![KeyPublisher](tutorials/moving_robot/keypublisher_data.png)

We want to map these keystrokes into messages of type `Twist` and publish them to the `/cmd_vel` topic which our model listens to.
The `TriggeredPublisher` plugin will do this.

### Triggered Publisher

The `TriggeredPublisher` plugin publishes a user specified message on an output topic in response to an input message that matches user specified criteria.
Let's add the following code under the `<world>` tags:

```xml
<!-- Moving Forward-->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

This code defines the `triggered-publisher` plugin.
It accepts messages of type `ignition.msgs.Int32` on the `/keyboard/keypress` topic and if the value in the `data` field matches `16777235`(Up arrow key) it outputs a `Twist` message on the `cmd_vel` topic with values `x: 0.5`, `z: 0.0`.

Now launch `building_robot.sdf` and our robot should move forward as we press the Up arrow key &#8593;.

There is a demo explaining how the [Triggered Publisher](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo2/tutorials/triggered_publisher.md) works.

### Moving using arrow keys

To see what values are sent on the `/keyboard/keypress` topic when pressing the arrows we can use the `--echo` or `-e` option

* Run the model in one terminal:

    `ign gazebo building_robot.sdf`

* In another terminal run the following command:

    `ign topic -e -t /keyboard/keypress`

Start pressing the arrows keys and see what values they give:

* Left &#8592;  : 16777234
* Up  &#8593;   : 16777235
* Right &#8594; : 16777236
* Down &#8595;  : 16777237

We will add the `Triggered publisher` plugin for each arrow key.
For example, the Down arrow:

```xml
<!-- Moving Backward-->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777237</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: -0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

Map each arrow (key stroke) with the desired message (movement) as we did with the backward arrow:

* Left &#10142; 16777234 &#10142; linear: {x: 0.0}, angular: {z: 0.5}
* Up &#10142; 16777235 &#10142; linear: {x: 0.5}, angular: {z: 0.0}
* Right &#10142; 16777236 &#10142; linear: {x: 0.0}, angular: {z: -0.5}
* Down &#10142; 16777237 &#10142; linear: {x: 0.5}, angular: {z: 0.0}

Now it's your turn try to make the robot move using different keys.

In the [next tutorial](sdf_worlds), you'll learn to create your own simulated world with SDF.
