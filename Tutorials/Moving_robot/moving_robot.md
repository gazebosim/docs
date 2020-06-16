# Moving our robot

In this tutorial we will learn how to move our robot. We will use the robot we built in [Build your own robot](SDF_tutorial_link) tutorial. You can download the robot from [here](car_demo.sdf). You can also found the finished world of this tutorial [here](move_robot.sdf).
To run the file open the terminal and write

`ign gazebo car_demo.sdf`

you should have your robot look like this

[!car_demo](screen_shot)

## What is a plugin

To make our robot move we will use the `diff_drive` plugin. But before doing so Let's answer the question "What is a plugin".A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. Plugins make us control many aspects of the simulation like world, models and etc.

### diff_drive plugin

`diff_drive` plugin help us to control our robot, specifically a robot that can be differentially driven. Let's setup the plugin on our robot. Open the `car_demo.sdf` in your favorite text editor and add the following code under the `<model>` tag.

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

#### Breaking the code down

The `<plugin>` tag has two attributes, `filename` which takes the library name and `name` takes the name of the plugin. In the `<left_joint>` and `<right_joint>` tags we define the joints which connect the left and right wheel with the body of the robot, in our case `left_wheel_joint` and `right_wheel_joint`. `<wheel_separation>` takes the distance between the two wheels in our robot, We have our `left_wheel` at 0.6 m and `right-wheel` at -0.6 m in y-axis with respect to the `chassis` so the `wheel_separation` is 1.2 m. `<wheel_radius>` takes the radius of the wheel which was defined in the `<radius>` tag under the wheel link. `<odom_publish_frequency>` set the frequency by which our car will accept the moving commands it is in `Hz` unit. `cmd_vel` is the input `<topic>` to the `DiffDrive` we cho

## Nodes, Topics and Messages

Now our model is ready. We just need to send commands(Messages) to it, These messages are published(sent) on the `cmd_vel` topic. A topic is simply a name for grouping a specific set of messages or a particular service. Our model will subscribe(listen) to the messages sent on the `cmd_vel`.

Launch the car world.

`ign gazebo car_demo.sdf`

In another terminal let's send a message to to our car.

`ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`

Now you should have your robot moving in the simulation. **Note:** don't forget to press the play in the simulation. The command is simple we need to specify the topic to publish to after the `-t` option, And after the `-m` we specify the message type. Our robot expect message of type `Twist` which consists of two components `linear` and `angular`. After the `-p` option we specify the content(value) of the message. for linear speed `x: 0.5` for angular speed `z: 0.05`.
For more information about `topics` and `messages` in ignition check Transport lib [tutorials](https://ignitionrobotics.org/api/transport/9.0/tutorials.html)

## Moving the robot using keyboard

Instead of sending messages from the terminal we will send messages using the arrows in the keyboard. To do so will add two new plugins `KeyPublisher` and `TriggeredPublisher`.

### KeyPublisher

`KeyPublisher` is an `ign-gui` plugin that reads keyboard's keystrokes and send them on a default topic `/gazebo/keyboard/keypress`. to use this plugin add the following code under the `<gui>` tag.

```xml
<!-- KeyPublisher plugin-->
<plugin filename="KeyPublisher" name="Key Publisher"/>
```

Let's try this plugin as follow:

* In one terminal type

    `ign gazebo car_demo.sdf`

* Start the simulation(play button at the bottom left)
* In another terminal type 

    `ign topic -e -t /gazebo/keyboard/keypress`
    
     this command will display all messages sent on `/gazebo/keyboard/keypress` topic.
* In the ignition window press different keys and should see numbers on the terminal where you run the

    `ign topic -e -t /gazebo/keyboard/keypress`

![KeyPublisher](KeyPublisher.png)

We want to map these key stokes into messages of type `Twist` and publish them to `/cmd_vel` topic which our model listens to. The `TriggeredPublisher` will do this.

### Triggered Publisher

The TriggeredPublisher system publishes a user specified message on an output topic in response to an input message that matches user specified criteria. Let's add the following code under the `<world>` tags

```xml
<!-- Moving Forward-->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/gazebo/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

This code defines `triggered-publisher` plugin, it accepts messages of type `ignition.msgs.Int32` on the `/gazebo/keyboard/keypress` topic and if the value in the `data` field matches "16777235"(Up arrow) it outputs a `Twist` message on the `cmd_vel` topic with value `x: 0.5`, `z: 0.0`.

Now launch the `car_demo.sdf` and our robot should move forward as you press the Up arrow key &#129045;

There is a demo explains how the [Triggered Publisher](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo2/tutorials/triggered_publisher.md) works.

### Moving using arrow keys

To see what values sent on the `/gazebo/keyboard/keypress` when pressing the arrows we can use `--echo` or `-e` option
Run the model in one terminal.
And in another terminal run the following command.

`ign topic -e -t /gazebo/keyboard/keypress`

Start press the arrows keys and see what values they give:

* Left &#8592;  : 16777234
* Up  &#8593;   : 16777235
* Right &#8594; : 16777236
* Down &#8595;  : 16777237

We will add `Triggered publisher` plugin for each arrows for example the backward, **Note:**You can use another keys to move your robot.

```xml
<!-- Moving Backward-->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/gazebo/keyboard/keypress">
        <match field="data">16777237</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: -0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

We mapped each arrow (key stroke) with the desired message(movement)

* Left &#10142; 16777234 &#10142; angular: linear: {x: 0.0}, angular: {z: 0.5}
* Up &#10142; 16777235 &#10142; linear: {x: 0.5}, angular: {z: 0.0}
* Right &#10142; 16777236 &#10142; linear: {x: 0.0}, angular: {z: -0.5}
* Down &#10142; 16777237 &#10142; linear: {x: 0.5}, angular: {z: 0.0}

## TODO

* think about remove introduction header? (done)
* Link to the tutorial and the sdf
* screen shot to the finished model
* link to Plugin future tutorial in the what is a plugin section
* remove the Breaking the code header? (I think ok)
* rewrite the topics and communication in different way (done)
* use either we or you pronoun 
* link to the triggered publisher tutorial (done)
* we can specify the name of the topic in the diff_drive_plugin(I think default is the name of the model) (done)
* maybe remove the triggered start example, just keep our problem (done)
* the blue color of the model in this workspace 
* you can customize keyboard plugin (later in the plugin tutorial) (contributing guide) (modify other plugins)
* [Arrows symbols](https://unicode-table.com/en/sets/arrow-symbols/) (done)
* screen shot, GIF of the output after every stage (eg. trying key plugin) or may do a video.
* rename the keypublisher plugin (done)
* Change link for triggered publisher, make it link to website after merge 
