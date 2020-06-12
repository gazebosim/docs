# Building your own robot

In this tutorial we will learn how to build our own in SDFormat. We will build a simple two wheeled robot which looks like this.

![two_wheeled_robot]()

 You can download the code of this tutorial from [here](car_demo).

## What is SDF?

Our model will need a world to spawn into but before explaining the world let's answer the question, "What is SDFormat?"<br/>
SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

## Building a world in SDF

We will start by this simple world and add our model to it. Let's have a look at the tags used in the `car_demo.sdf` to define the world.

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
```

The file starts with defining the XML and SDF versions to be used. Any SDF file should contain this `<sdf>` tag, everything else goes inside this tag.<br/>
**Note**: Some features in this tutorial are not available in older sdf versions.<br/>

```xml
<world name="car_world">
```

Then we have the `<world>` tag with the name `car_world` to describe our world. It can contain any of the following tags and more:

* `<physics>`: Specifies the type and properties of the dynamics engine.
* `<plugin>`: It is a dynamically loaded chunk of code.
* `<gui>`: Describes how the GUI looks like when ignition is loaded.
* `<light>`: Describes a light source.
* `<model>`: Defines a complete robot or any other physical object.

We will explain them in more details in the following tutorials.

## Building our model

Inside the `<world> </world>` tags we will start to build our model. Let's break the code down.

### Defining the model

```xml
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to=world>0 0 0 0 0 0</pose>
```

Here we define the name of our model `vehicle_blue` Which should be a unique name among its sibling on the same level(other tags or models). Each non-static model must have at least one link designated as the `canonical_link` The implicit frame of the model is attached to this link. The `<pose>` tag is used to define the position and orientation of our model and the `relative_to` attribute is used to define the frame of the model relative to any other frame, if the `relative_to` is not defined the link `<pose>` will be relative to the `canonical_link`. Let's make our pose relative to the `world`. The values inside the pose tag are as follow `<pose>X Y Z R P Y</pose>` where the `X Y Z` represent the position of the frame and `R P Y` represent the orientation in roll pitch yaw. We set them to zeros which makes the two frames identical.<br/> Every model is a group of "links"(can be just one link) connected together with "joints".

### Links forming our robot

### Building chassis

```xml
<link name='chassis'>
    <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
```

We define the first link, the `chassis` of our car and it's pose relative to the model.

#### Inertial properties

```xml
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
```

Here we define the inertial properties of chassis like the mass and the inertia matrix. The values of the inertia matrix for primitive shapes can be found [here](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&#joint)

#### Creating visual and collision

```xml
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
```

As the name suggests `<visual>` tag is responsible for how our link will look like.
We define the shape of our link inside the `<geometry>` tag as a `<box>`(cuboid) and then specify the three dimensions(in meters) of this box inside the `<size>` tag, Then inside the `<material>` tag we define the material of our link. Here we defined the `<ambient>`, `<diffuse>` and `<specular>` colors in a set of four numbers red/green/blue/alpha each in range [0, 1].

```xml
<collision name='collision'>
    <geometry>
        <box>
            <size>2.0 1.0 0.5</size>
        </box>
    </geometry>
</collision>
```

The `<collision>` tag define the collision properties of the link(how our link will react with other elements and the effect of the physics engine on it).<br/>
**Note**: `<collision>` can be different from the visual properties, for example, simpler collision models are often used to reduce computation time.

Run the world to see it using this command `ign gazebo demo.sdf`
Our model should look like this.

![car chassis]()

In the top right corner click on the plugins button(vertical ellipsis), choose `Transform control`, select your model and then click on the Translation tool you should see three axis like this

![model_axis]()

These are the axis of our model where the red is x-axis, green is the y-axis and blue is the z-axis.

### Building left wheel

```xml
<link name='left_wheel'>
    <pose>-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>2</mass>
        <inertia>
            <ixx>0.145833</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.145833</iyy>
            <iyz>0</iyz>
            <izz>0.125</izz>
        </inertia>
    </inertial>
```

We define the name of our link `left_wheel` and then defined its `<pose>` we didn't use `relative_to`, in this case the default value is the `canonical_link` which is the `chassis` in our case. The wheel need to be placed on the left to the back of the `chassis` so that's why we chose these values for `pose` as `-0.5 0.6 0`. Also our wheel is a cylinder but on its side so that's why we defined the orientation value as `-1.5707 0 0` which is a -90 degree rotation around the x-axis(the angles are defined in radians).
Then we defined the `inertial` properties of the wheel the `mass` and the `inertia` matrix.

#### Left wheel visualization

```xml
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
```

The `<visual>` and  the `<collision>` properties are similar to the previous link except for the shape of our link it has the shape of `<cylinder>` that require two dimensions the `<radius>` and `<length>`.
Our model should look like this.

![this](left_wheel)

### Building right wheel

```xml
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose> <!--angles are in radian-->
```

The right wheel is similar to the left wheel except for position of its frame.
Our model should look like this.

![this](left_wheel)

### Defining an arbitrary frame

```xml
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

One of the new features of sdf 1.7 is that we can define arbitrary frames. It takes two attributes:

* `name`: the name of the frame
* `attached_to`: the name of the frame or the link to which this frame is attached.

We gave our link name `caster_frame` and attached it to the `chassis` link. Then the `<pose>` tag to define the position and orientation of the frame, we didn't use the `relative_to` so the pose is with respect to the frame named in the `attached_to` attribute `chassis` in our case.

### Building caster wheel

```xml
<link name='caster'>
    <pose relative_to='caster_frame'/>
```

Our last link is the `caster` and its pose is with respect to the frame `caster_frame` we defined above. As you could notice we closed the `pose` tag without defining the position or the orientation, in this case the pose of the link is the same as(identity) the frame in `relative_to`.

#### Visualizing caster wheel

```xml
<geometry>
    <sphere>
        <radius>0.2</radius>
    </sphere>
</geometry>
```

In the caster link we defined a different shape `<sphere>` which require the `<radius>` of the sphere.Right now we have our robot like this.

![Simple two wheeled robot]()

### Connecting links together(joints)

We need to connect these links together and here comes the job of `<joint>` tag. The joint tag connect two links together and define how they will move with respect to each other. Inside the `<joint>` tag we need to define the two links to connect and their relation.

#### Left wheel joint

```xml
<joint name='left_wheel_joint' type='revolute'> <!--continuos joint is not supported yet-->
    <pose relative_to='left_wheel'/>
```

Our first joint is the `left_wheel_joint`. It takes two attributes the name `name='left_wheel_joint'` and the type `type='revolute'`. revolute type gives 1 rotational degree of freedom with joint limits. And the pose of the joint is the same as `left_wheel` frame.

```xml
<parent>chassis</parent>
<child>left_wheel</child>
```

Every joint connect two links(bodies) together. Here we connect the `chassis` with the `left_wheel`. `chassis` is the parent link and `left_wheel` is the child link.

```xml
<axis>
    <xyz expressed_in='chassis'>0 1 0</xyz> <!--can be descried to any frame even an arbitrary frames-->
    <limit>
        <lower>-1.79769e+308</lower>    <!--negative infinity-->
        <upper>1.79769e+308</upper>     <!--positive infinity-->
    </limit>
</axis>
```

Here we define the axis of rotation. the axis of rotation can be any frame not just the `parent` or the `child` link. We chose the y-axis with respect to the `chassis` frame so we put 1 in the y element and zeros in the other. For the revolute joint we need to define the `<limits>` of our rotation angle in the `<lower>` and `<upper>` tags.
**Note**: The angles are in radians.

#### Right wheel joint

The `right_wheel_joint` is very similar except for the pose of the joint and this joint connect the `right_wheel` with the `chassis`.

#### Caster wheel joint

```xml
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
```

For the caster we need different type of joint(connection). we used `type='ball'` which gives 3 rotational degrees of freedom.

## Conclusion 
Run the world to see it using this command `ign gazebo car_demo.sdf`. It should look like this <br/>

![two_wheeled_robot]()

Hurray we build our first robot. You can know more details about SDFormat tags [here][http://sdformat.org/spec]. In the next [tutorial](moving_robot_demo) we will learn how to move our robot around.

### TODO

* link to the world tutorials
