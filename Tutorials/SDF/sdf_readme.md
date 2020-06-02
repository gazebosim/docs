# SDF tutorial

In this tutorial we will learn how to build our own in SDFormat. We will build a simple two wheeled two robot which looks like this.
![img]()
To build our model we need a world to spawn our model into it. We can use the empty world example and add our model into it. You can dowload it from [here](https:) or copy and paste the text into your text editor.

todo: put code snippet here 

#### SDF file

before explaining the empty world let's say What is SDFormat?
SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

Breaking the code down

~~~
<?xml version="1.0" ?>
<sdf version="1.7">
~~~

The file starts with defining the xml and sdf versions to be used. Any sdf file should contain this sdf tags. Everything else goes inside those tags.<br/>
**Note**: Some features in this tutorial are not availabe in older sdf versions.<br/>

~~~
<world name="car_world">
~~~

Then we have the world tag with the name car_world to describe our world. It can contain any of these tags and more.

* `<physics>`: Specifies the type and properties of the dynamics engine.
* `<plugin>`: It is a dynamically loaded chunk of code.
* `<gui>`: Descibes how the GUI looks like when ignition is loaded.
* `<light>`: Descibes a light source.
* `<model>`: Defines a complete robot or any other physical object.

inside the world tags we will start to build our model as follow 

Let's break the code down

~~~
<model name='vehicle_blue' canonical_link='chassis'>
            <pose relative_to=world>0 0 0 0 0 0</pose>
~~~

Here we define the name of our model `vehicle_blue` Which should be a unique name among its sibling on the same level. `canonical_link` attribute defines the link which will be used as the frame when reffering to the model frame. (I need to redfine this sentence). The pose tag is used to define the position and orientation of our model. here the `relative_to` attirbute is used to define the co-ordinate frame of the model relative to any other co-ordinate frame. make our pose relative to the `world`. The values inside the pose tag are as follow `<pose>X Y Z R P Y</pose>` where the `X Y Z` represent the positon of the frame and `R P Y` represent the orientation in roll pitch yaw. Here we set them to zeros which makes the two frames identical.

Let's build the links of our car
~~~
<link name='chassis'>
    <pose relative_to=__model__>1 1 1 0 0 0</pose>
~~~
We define the chassis of our car and it's pose relative to the model. 

~~~
<inertial> <!--inertial properties of the link mass, inertia matix-->
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
Here we define the inetial properties of chassis like the mass and the intertia matrix. the values of the inertia matrix for primsitive shapes can be found [here](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&#joint)

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
As the name suggestes `visual` tag is responsible for how our link would look like.
we define its shape inside the geometery tag as a `box`(cuboid) and then specify the three dimensions(in meters) of this box inside the `size` tag. Then inside the material tag we define the material of our link. Here we defined the `ambient`, `diffuse` and `specular` colors in a set of four numbers red/green/blue/alpha each in range [0, 1]. (what does alpha represent, I think transperency)

~~~
<collision name='collision'> 
    <geometry>
        <box>
            <size>2.0 1.0 0.5</size>
        </box>
    </geometry>
</collision>
~~~
The `<collison>` tag define the collision properties of the link. Which define how our link will react with other elements and the effect of the physics enginee on it.
**Note**: It can be different from the visual properties, for example, simpler collison models are often used to reduce computation time.

Let's build the left wheel
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

We define the name of our link `left_wheel` and then defined its pose relative to the chassis(todo maybe explain the values). Our wheel is a cylinder but on its side so that's why we defined the orientation value as `-1.5707 0 0` which is a -90 degree rotation around the x-axis(the angles are defined in radians).
Then we defined the inertial properties of the wheel the mass and the inertia matrix

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
The `visual` and `collision` properties are similar to the previous link except for the shape of our link it has the shape of `<cylinder>` it require two dimensiions the `<radius>` and `<length>`. 

~~~
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose> <!--angles are in radian-->
~~~

The right wheel is similar to the left wheel except for position of its frame.

Our model should look like ![this]()

~~~
<frame name="F" attached_to='chassis'>
    <pose>0.8 0 -0.45 0 0 0</pose>
</frame>
~~~
One of the new features of sdf 1.7 is that we can define arbittray frames. It takes two attributes:
* name: the name of the frame
* attached_to: the name of the frame or the link to which this frame is attached.
so we gave our robot name `F` and attached it to the chassis link. Then pose tag to define the positon and orientation of the frame. we didn't use the `relative_to` so the pose is with respect to the frame named in the relative_to attribute `chassis` in our case.

~~~
<link name='caster'>
    <pose relative_to='F'/>
~~~
Our last link with name caster and its pose is defined relative to the frame `F` we defined above. As you could notice we closed the `pose` tag without defining the position or the orientation in this case the pose of the link is the same as the frame in `relative_to`(identity).

~~~
<geometry>
    <sphere>
        <radius>0.2</radius>
    </sphere>
</geometry>
~~~
In the caster link we defined a different shape `<sphere>` and then define the `<radius>` of the sphere.

Right now we have our robot like ![this]()
We need to connect these links together and here comes the job of `<joint>` tag. The joint tag connect two links together and define how they will move with repect to each other.

~~~
<joint name='left_wheel_joint' type='revolute'> <!--continous joint is not supported yet-->
    <pose relative_to='chassis'>-0.5 0.5 0 0 0 0</pose>
~~~
Our first joint is the left_wheel_joint. It takes two attributes the name `name='left_wheel_joint'` and the type `type='revolute'`. revolute type gives 1 rotational degree of freedom with joint limits.
The pose is with respect to the chassis.

~~~
<parent>chassis</parent>
<child>left_wheel</child>
~~~
Every joint connect two links(bodies) together. Here we connect the chassis with the left wheel. `chassis` is the parent link and `left_wheel` is the child link.

~~~
<axis>
    <xyz expressed_in='chassis'>0 1 0</xyz> <!--can be descired to any frame even an arbitrary frames-->
    <limit>
        <lower>-1.79769e+308</lower>    <!--negative infinity-->
        <upper>1.79769e+308</upper>     <!--positive infinity-->
    </limit>
</axis>
~~~

Here we define the axis of rotation. the axis of rotation can be any frame not just the parent or the child link. we choose the y-axis with respect to the `chassis` frame. For the revolute joint we need to define the limits of our rotation angle in the `<lower>` and `<upper>` tags. The angles are in radians.

The right_wheel_joint is very similar except for the pose of the joint and this joint connect the right_wheel with the chassis.

~~~
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
~~~
For the caster we need different type of joint(connection). we used `type='ball'` which gives 3 rotational degrees of freedom.

Hurray we build our first robot. You can know more details about SDFormat tags [here][http://sdformat.org/spec] 