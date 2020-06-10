# Building your own robot

## Introduction

In this tutorial we will learn how to build our own in SDFormat. We will build a simple two wheeled two robot which looks like this. You can download the code of this tutorial from [here](car_world).<br/>

## Building the world

Our model will also need a world to spawn into. We will start by this simple world and add our model to it.

~~~
<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin
            filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin
            filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="libignition-gazebo-contact-system.so"
            name="ignition::gazebo::systems::Contact">
        </plugin>

        <gui fullscreen="0">

            <!-- 3D scene -->
            <plugin filename="GzScene3D" name="3D View">
                <ignition-gui>
                <title>3D View</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="string" key="state">docked</property>
                </ignition-gui>

                <engine>ogre2</engine>
                <scene>scene</scene>
                <ambient_light>0.4 0.4 0.4</ambient_light>
                <background_color>0.8 0.8 0.8</background_color>
            </plugin>

            <!-- World control -->
            <plugin filename="WorldControl" name="World control">
                <ignition-gui>
                <title>World control</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="bool" key="resizable">false</property>
                <property type="double" key="height">72</property>
                <property type="double" key="width">121</property>
                <property type="double" key="z">1</property>

                <property type="string" key="state">floating</property>
                <anchors target="3D View">
                    <line own="left" target="left"/>
                    <line own="bottom" target="bottom"/>
                </anchors>
                </ignition-gui>

                <play_pause>true</play_pause>
                <step>true</step>
                <start_paused>true</start_paused>
                <service>/world/empty/control</service>
                <stats_topic>/world/empty/stats</stats_topic>

            </plugin>

            <!-- World statistics -->
            <plugin filename="WorldStats" name="World stats">
                <ignition-gui>
                <title>World stats</title>
                <property type="bool" key="showTitleBar">false</property>
                <property type="bool" key="resizable">false</property>
                <property type="double" key="height">110</property>
                <property type="double" key="width">290</property>
                <property type="double" key="z">1</property>

                <property type="string" key="state">floating</property>
                <anchors target="3D View">
                    <line own="right" target="right"/>
                    <line own="bottom" target="bottom"/>
                </anchors>
                </ignition-gui>

                <sim_time>true</sim_time>
                <real_time>true</real_time>
                <real_time_factor>true</real_time_factor>
                <iterations>true</iterations>
                <topic>/world/empty/stats</topic>

            </plugin>

            <!-- Entity tree -->
            <plugin filename="EntityTree" name="Entity tree">
            </plugin>

        </gui>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
    </world>
</sdf>
~~~

## What is SDF?

Before explaining the world let's answer the question, "What is SDFormat?"
SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

## Building a world in SDF

Let's have a look at the common tags used in the `empty.sdf` to define the world.
~~~
<?xml version="1.0" ?>
<sdf version="1.7">
~~~

The file starts with defining the xml and sdf versions to be used. Any sdf file should contain this sdf tag, everything else goes inside this tag.<br/>
**Note**: Some features in this tutorial are not available in older sdf versions.<br/>

~~~
<world name="car_world">
~~~

Then we have the `<world>` tag with the name `car_world` to describe our world. It can contain any of the following tags and more:

* `<physics>`: Specifies the type and properties of the dynamics engine.
* `<plugin>`: It is a dynamically loaded chunk of code.
* `<gui>`: Describes how the GUI looks like when ignition is loaded.
* `<light>`: Describes a light source.
* `<model>`: Defines a complete robot or any other physical object.

We will explain them in more details in the following tutorials.

## Building our model

Inside the `<world> </world>` tags we will start to build our model as follows. You can download the code from [here](car_world). Let's break the code down.

### Defining the model

~~~
<model name='vehicle_blue' canonical_link='chassis'>
            <pose relative_to=world>0 0 0 0 0 0</pose>
~~~

Here we define the name of our model `vehicle_blue` Which should be a unique name among its sibling on the same level(other tags or models). `canonical_link` attribute defines the link which will be used when referring to the model frame. (todo: I need to redefine this sentence). The `<pose>` tag is used to define the position and orientation of our model. Here the `relative_to` attribute is used to define the frame of the model relative to any other frame. Let's make our pose relative to the `world`. The values inside the pose tag are as follow `<pose>X Y Z R P Y</pose>` where the `X Y Z` represent the position of the frame and `R P Y` represent the orientation in roll pitch yaw. We set them to zeros which makes the two frames identical.<br/> The model is a group of links(can be just one link) connected together with joints.

### Links forming our robot

~~~
<link name='chassis'>
    <pose relative_to=__model__>1 1 1 0 0 0</pose>
~~~

We define the first link, the `chassis` of our car and it's pose relative to the model.

#### Inertial properties

~~~
<inertial> <!--inertial properties of the link mass, inertia matrix-->
    <mass>1.2</mass>
    <inertia>
        <ixx>0.125</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.425</iyy>
        <iyz>0</iyz>
        <izz>0.5</izz>
    </inertia>
</inertial>
~~~

Here we define the inertial properties of chassis like the mass and the inertia matrix. The values of the inertia matrix for primitive shapes can be found [here](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&#joint)

#### Creating visual and collision

~~~
<visual name='visual'>
    <geometry>
        <box>
            <size>2.0 1.0 0.5</size>
        </box>
    </geometry>
    <!--let's add color to our link-->
    <material>
        <ambient>0.0 0.0 1.0 1</ambient>
        <diffuse>0.0 0.0 1.0 1</diffuse>
        <specular>0.0 0.0 1.0 1</specular>
    </material>
</visual>
~~~

As the name suggests `<visual>` tag is responsible for how our link will look like.
We define the shape of our link inside the `<geometry>` tag as a `<box>`(cuboid) and then specify the three dimensions(in meters) of this box inside the `<size>` tag, Then inside the `<material>` tag we define the material of our link. Here we defined the `<ambient>`, `<diffuse>` and `<specular>` colors in a set of four numbers red/green/blue/alpha each in range [0, 1].

~~~
<collision name='collision'>
    <geometry>
        <box>
            <size>2.0 1.0 0.5</size>
        </box>
    </geometry>
</collision>
~~~

The `<collision>` tag define the collision properties of the link(how our link will react with other elements and the effect of the physics engine on it).<br/>
**Note**: `<collision>` can be different from the visual properties, for example, simpler collision models are often used to reduce computation time.

Run the world to see it using this command `ign gazebo car_world.sdf`
Our model should look like this.

![car chassis]()

### Building left wheel

~~~
<link name='left_wheel'>
    <pose relative_to='chassis'>-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.0433</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0433</iyy>
            <iyz>0</iyz>
            <izz>0.0133</izz>
        </inertia>
    </inertial>
~~~

We define the name of our link `left_wheel` and then defined its `<pose>` relative to the `chassis`, we need our wheel on the left to the back of the `chassis` so that's why we chose these values for `pose` as `-0.5 0.6 0`. Also our wheel is a cylinder but on its side so that's why we defined the orientation value as `-1.5707 0 0` which is a -90 degree rotation around the x-axis(the angles are defined in radians).
Then we defined the `inertial` properties of the wheel the `mass` and the `inertia` matrix.

#### Left wheel visualization

~~~
<visual name='visual'>
    <geometry>
        <cylinder>
            <radius>0.4</radius> 
            <length>0.2</length>
        </cylinder>
    </geometry>
    <material>
        <ambient>1.0 0.0 0.0 1</ambient>
        <diffuse>1.0 0.0 0.0 1</diffuse>
        <specular>1.0 0.0 0.0 1</specular>
    </material>
</visual>
<collision name='collision'>
    <geometry>
        <cylinder>
            <radius>0.4</radius>
            <length>0.2</length>
        </cylinder>
    </geometry>
</collision>
~~~

The `<visual>` and  the `<collision>` properties are similar to the previous link except for the shape of our link it has the shape of `<cylinder>` that require two dimensions the `<radius>` and `<length>`.
Our model should look like this.

![this](left_wheel)

### Building right wheel

~~~
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose> <!--angles are in radian-->
~~~

The right wheel is similar to the left wheel except for position of its frame.
Our model should look like this.

![this](left_wheel)

### Defining arbitrary frame

~~~
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.45 0 0 0</pose>
</frame>
~~~

One of the new features of sdf 1.7 is that we can define arbitrary frames. It takes two attributes:

* `name`: the name of the frame
* `attached_to`: the name of the frame or the link to which this frame is attached.

We gave our link name `caster_frame` and attached it to the `chassis` link. Then the `<pose>` tag to define the position and orientation of the frame. We didn't use the `relative_to` so the pose is with respect to the frame named in the `relative_to` attribute `chassis` in our case.

### Building caster wheel

~~~
<link name='caster'>
    <pose relative_to='caster_frame'/>
~~~

Our last link with name `caster` and its pose is with respect to the frame `caster_frame` we defined above. As you could notice we closed the `pose` tag without defining the position or the orientation, in this case the pose of the link is the same as (identity)the frame in `relative_to`.

#### Visualizing caster wheel

~~~
<geometry>
    <sphere>
        <radius>0.2</radius>
    </sphere>
</geometry>
~~~

In the caster link we defined a different shape `<sphere>` which require the `<radius>` of the sphere.

Right now we have our robot like this.

![Simple two wheeled robot]()

todo: I think one image of full model is enough remove one at the top or remove this.

### Connecting links together(joints)

We need to connect these links together and here comes the job of `<joint>` tag. The joint tag connect two links together and define how they will move with respect to each other. Inside the `<joint>` tag we need to define the two links to connect and their relation.

#### Left wheel joint

~~~
<joint name='left_wheel_joint' type='revolute'> <!--continuos joint is not supported yet-->
    <pose relative_to='chassis'>-0.5 0.5 0 0 0 0</pose>
~~~

Our first joint is the `left_wheel_joint`. It takes two attributes the name `name='left_wheel_joint'` and the type `type='revolute'`. revolute type gives 1 rotational degree of freedom with joint limits. And the pose of the joint is with respect to the `chassis`.

~~~
<parent>chassis</parent>
<child>left_wheel</child>
~~~

Every joint connect two links(bodies) together. Here we connect the `chassis` with the `left_wheel`. `chassis` is the parent link and `left_wheel` is the child link.

~~~
<axis>
    <xyz expressed_in='chassis'>0 1 0</xyz> <!--can be descried to any frame even an arbitrary frames-->
    <limit>
        <lower>-1.79769e+308</lower>    <!--negative infinity-->
        <upper>1.79769e+308</upper>     <!--positive infinity-->
    </limit>
</axis>
~~~

Here we define the axis of rotation. the axis of rotation can be any frame not just the `parent` or the `child` link. We chose the y-axis with respect to the `chassis` frame so we put 1 in the y element and zeros in the other. For the revolute joint we need to define the `<limits>` of our rotation angle in the `<lower>` and `<upper>` tags.
**Note**: The angles are in radians.

#### Right wheel joint

The right_wheel_joint is very similar except for the pose of the joint and this joint connect the right_wheel with the chassis.

#### Caster wheel joint

~~~
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
~~~

For the caster we need different type of joint(connection). we used `type='ball'` which gives 3 rotational degrees of freedom.

## Conclusion 
Run the world to see it using this command `ign gazebo car_world.sdf` <br/>
Hurray we build our first robot. You can know more details about SDFormat tags [here][http://sdformat.org/spec]. In the next tutorial we will learn how to move our robot around.