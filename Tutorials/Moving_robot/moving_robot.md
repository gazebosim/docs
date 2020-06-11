# Moving our robot

## Introduction

todo: think about remove introduction title
In this tutorial we will learn how to move our robot. We will use the robot we built in [Build your own robot](SDF_tutorial_link) tutorial. You can download the robot from [here](car_world.sdf).
To run the file open the terminal and write `ign gazebo car_world.sdf`
you should have your robot look like this

[!car_world](screen_shot)

## What is a plugin

To make our robot move we will use `diff_drive` plugin. But before doing so Let's answer the question "What is a plugin".A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. Plugins make us control many aspects of the simulation the world, models and etc.

### diff_drive plugin

`diff_drive` plugin help us to control our robot, specifically a robot that can be differentially driven. Let's setup the plugin on our robot. Open the `car_world.sdf` in your favorite text editor and add the following code inside the `<model>` tag.

```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
</plugin>
```

#### Breaking the code down

The `<plugin>` tag has two attributes, `filename` which takes the library name, `name` takes the name of the plugin. In the `<left_joint>` and `<right_joint>` tags we define the joints which connect the left and right wheel with the body of the robot, in our case `left_wheel_joint` and `right_wheel_joint`. `<wheel_separation>` takes the distance between the two wheels in our robot, We have our `left_wheel` at 0.6 and `right-wheel` -0.6 in y-axis with respect to the `chassis`. `<wheel_radius>` takes the radius of the wheel which was defined in the `<radius>` tag. `<odom_publish_frequency>` set the frequency by which our car will accept the moving commands it is in `HZ` unit

## Messages and Topics

One of the main concepts in communication in Ignition is "Topics". A topic is simply a name for grouping a specific set of messages or a particular service. The communication happens between two nodes through topics. All the nodes in the network can act as publishers, subscribers, provide services and request services. A publisher is a node that produces information and a subscriber is a node that consumes information.<br/>
In our case we will send a message to topic name `/model/vehicle_blue/cmd_vel` and our robot subscribe to this topic to get command that make the robot move. Let's do it<br/> 
Launch the car world `ign gazebo car_world.sdf`
In another terminal let's send a message to to our car.<br/>
`ign topic -t "/model/vehicle_blue/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`
Now you should have your robot moving in the simulation **Note** don't forget to press the play in the simulation. The command is simple we need to specify topic to publish to after the `-t` option and after the `-m` we specify the message type. Our robot expect message of type `Twist` which consists of two components `linear` and `angular`. after the `-p` option we specify the content(value) of the message. for linear speed `x: 0.5` for angular speed `z: 0.05`.
For more information about topics and messages in ignition check Transport lib [tutorials](igniton transport)

## Moving the robot using keyboard

Instead of making our robot move in a specific speed we will control our robot using two plugins

### Keystrokes 

key strokes

### Triggered Publisher

The TriggeredPublisher system publishes a user specified message on an output topic in response to an input message that matches user specified criteria. Add the following code inside the `<world>` tag

```xml
```

For more details this tutorial describes how the [Triggered Publisher](triggered publisher md) works

## TODO

* think about remove introduction header?
* Link to the tutorial and the sdf
* screen shot to the finished model
* link to Plugin future tutorial in the what is a plugin section
* remove the Breaking the code header?
* rewrite the topics and communication in different way
* use either we or you pronoun
* link to the triggered publisher tutorial
* we can specify the name of the topic in the diff_drive_plugin(I think default is the name of the model)
