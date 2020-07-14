# Actors demo

In this tutorial we will learn how to use `actors` to create a scripted animation.

Animations are very useful if we want to have entities following a predefined path in simulation without being affected by the physics. This means they won't fall due to gravity or collide with other objects. They will however, have a 3D visualization which can be seen by RGB cameras, and 3D meshes which can be detected by GPU based sensors. The tutorial explains how to create open-loop trajectories which don't interact with the rest of the simulation.

## Actors

In Ignition Gazebo, an animated model is called an `actor`. Actors extend common models, adding animation capabilities.

There are two types of animations which can be used separately or combined together:

* Skeleton animation, which is relative motion between links in one model:

![skeleton_movement](skeleton_movement)

* Trajectory animation, which carries all of the actor's links around the world as one group along a trajectory:

![trajectory_movement](trajectory_movement)

* Combined, to achieve a skeleton animation which moves in the world:

![combined_movement](combined_movement)

Actors are just like models, so you can put links and joints inside them as usual. The main differences are:

* Actors are always static (i.e. no forces are applied on them, be it from gravity or contact or anything else)
* Actors support skeleton animation imported from COLLADA(.dae) and BVH(.bvh) files.
* Actors can have trajectories scripted directly in SDF.
* There can't be models nested inside actors, so we're limited to animated meshes, links and joints.

You can check out the full specification for the `<actor>` SDF element [here](http://sdformat.org/spec?ver=1.6&elem=actor).

## Skeleton

Ignition gazebo supports two different skeleton animation file formats: COLLADA (.dae) and Biovision Hierarchy (.bvh).

We can create an actor called `actor_walking` as follows:

```xml
<actor name="actor_walking">
    <skin>
        <filename>https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
        <scale>1.0</scale>
    </skin>
</actor>
```

### Skin

In the `<skin>` tag we just loaded a COLLADA file `walk.dae` which specify how our actor will look like. When a COLLADA file used within the `<skin>` tags its animation is loaded. Now run the world and should see our model moving. The `<scale>` scales the skin's size.

**Note**: You can find different actors and models on ignition [fuel](https://app.ignitionrobotics.org/fuel).

### Animation

In the `<animation>` tag we specify how our actor will move. We can combine different skin with different animation as long as they have compatible skeletons.

For example the `moonwalk.dae` and `walk.dae` are compatible so they can be mixed with each other. The actor walking is with a green shirt and the one moon walking is with brown shirt. We can have an actor walking with a brown shirt by using the `moonwalk.dae` in the skin and the `walk.dae` in the animation as follow:

```xml
<actor name="actor_walking">
    <skin>
        <filename>https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor/tip/files/meshes/moonwalk.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="walk">
        <filename>https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    </animation>
</actor>
```

## Scripted trajectory

This is the high level animation of actors, which consists of specifying a series of poses to be reached at specific times. Ignition gazebo takes care of interpolating the motion between them so the movement is fluid.

We can make our actor follows the specified trajectory forever and to start playing as soon as the world is loaded as follows:

```xml
<script>
    <loop>true</loop>
    <delay_start>0.000000</delay_start>
    <auto_start>true</auto_start>
```

The script is defined between the `<actor>` `</actor>` tags.

Inside the `<script>` tag the following parameters are available:

* `loop`: Set this to true for the script to be repeated in a loop. For a fluid continuous motion, make sure the last waypoint matches the first one, as we'll do below.

* `delay_start`: This is the time in seconds to wait before starting the script. If running in a loop, this time will be waited before starting each cycle.

* `auto_start`: Set to true if the animation should start as soon as the simulation starts playing. It is useful to set this to false if the animation should only start playing only when triggered by a plugin, for example.

Let's define the trajectory as a sequence of waypoints:

```xml
        <trajectory id="0" type="walk">
            <waypoint>
                <time>0</time>
                <pose>0 0 1.0 0 0 0</pose>
            </waypoint>
            <waypoint>
                <time>2</time>
                <pose>2.0 0 1.0 0 0 0</pose>
            </waypoint>
            <waypoint>
                <time>2.5</time>
                <pose>2 0 1.0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
                <time>4</time>
                <pose>2 2 1.0 0 0 1.57</pose>
            </waypoint>
            <waypoint>
                <time>4.5</time>
                <pose>2 2 1.0 0 0 3.142</pose>
            </waypoint>
            <waypoint>
                <time>6</time>
                <pose>0 2 1 0 0 3.142</pose>
            </waypoint>
            <waypoint>
                <time>6.5</time>
                <pose>0 2 1 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>8</time>
                <pose>0 0 1.0 0 0 -1.57</pose>
            </waypoint>
            <waypoint>
                <time>8.5</time>
                <pose>0 0 1.0 0 0 0</pose>
            </waypoint>
        </trajectory>
    </script>
</actor>
```

Within the `<trajectory>` tag we can define a series of waypoints which our actor will follow. The `<trajectory>` has two attributes: `id` and `type`. The `type` should have the same name as the animation `walk`. The `trajectory` parameters are as follow:

* `waypoint`: There can be any number of waypoints in a trajectory. Each waypoint consists of a time and a pose:
    * `time`: The time in seconds, counted from the beginning of the script, when the pose should be reached.
    * `pose`: The pose which should be reached.

Run the world and you should see our actor moving in a square following the waypoints.

**Notes**:

* The order in which waypoints are defined is not important, they will follow the given times.
* The trajectory is smoothed as a whole. This means that you'll get a fluid motion, but the exact poses contained in the waypoints might not be reached.

Now it's your turn! Try out different trajectories description.

## TODO

* adding actors to our environment (fuel)
* move the actor with script
* closed loop control using plugin
* remove the sensor plugin from the sdf?
* use an already built world and add to it or start new?
* comment blocks in sdf
* GIF for skeleton and trajectory movement and combined
* when link to code link to the raw version.
* Make the movement reasonable
* refer to the final sdf of the tutorial
* make the move smoother if possible
* I think trajectory motion is not supported. but I can make it with the (talk_b.dae)