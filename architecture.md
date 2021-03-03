# Ignition Gazebo Architecture

[Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo) is an application entry-point of Ignition, generally encompassing the use of all other Ignition libraries.
As an executable, it runs a simulation by launching two processes: the backend server process and frontend client process.

The core functionality of Ignition Gazebo is the client-server communication and the various plugins it loads to facilitate the simulation.
Plugins are libraries that are loaded at runtime.
They allow Ignition Gazebo to utilize other Ignition libraries.
There are many plugin types in the Ignition framework.
For example, there are plugins that introduce new physics engines or rendering engines.
However, for the purpose of this document, any mention of plugins is referring only to Ignition Gazebo server and GUI plugins.

Because they’re loaded at runtime, Ignition does not need to be recompiled to add or remove plugins.
Ignition Gazebo ships with many plugins by default ([Server plugins](https://ignitionrobotics.org/api/gazebo/4.5/namespaceignition_1_1gazebo_1_1systems.html), [Ignition GUI plugins](https://ignitionrobotics.org/api/gui/4.2/namespaceignition_1_1gui_1_1plugins.html), [Ignition Gazebo GUI plugins]( https://ignitionrobotics.org/api/gazebo/4.5/namespaceignition_1_1gazebo.html), and more), all of which are optional and can be removed by the user.
Users can also add more plugins and even write their own plugins that will be compiled into library files.  

Ignition libraries are modular.
Plugins let Ignition Gazebo utilize other libraries.
For example, [Ignition Physics](https://ignitionrobotics.org/libs/physics) and Ignition Gazebo are independent of one another, so Ignition Gazebo has a physics plugin that uses Ignition Physics as a library and incorporates Gazebo specifics, allowing Physics to be a system running in the simulation loop.

The Ignition Physics library is only used in the physics plugin, not in other plugins nor in the core of Ignition Gazebo.
Some libraries are only used by one plugin, or one in each process (frontend and backend).
However, some foundational libraries are used by all plugins, like [Ignition Common](https://ignitionrobotics.org/libs/common) which provides common functionality like logging, manipulating strings, file system interaction, etc., to all plugins.
Other such libraries include [Ignition Plugin](https://ignitionrobotics.org/libs/plugin), [Ignition Math](https://ignitionrobotics.org/libs/math), [SDFormat](https://ignitionrobotics.org/libs/sdformat), and more.

There are certain plugins in both the frontend and backend processes that are loaded by default in every version of Ignition.
Many other plugins are optional.
One common optional plugin is the sensors plugin.
Both optional and default plugins can be removed or added at any time; Ignition Gazebo will continue to run with limited functionality.
These demos on [Server Configuration](https://ignitionrobotics.org/api/gazebo/4.3/server_config.html) and [GUI Configuration](https://ignitionrobotics.org/api/gazebo/4.3/gui_config.html) showcase that functionality.

The simulation process is depicted in the diagram below, and further explained in the Backend and Frontend process sections that follow.

![Ignition Gazebo architecture diagram](images/IgnitonGazeboArchicteture.png)

## Backend server process

Ignition Gazebo is responsible for loading plugins in the backend, referred to as systems.
The server runs an entity-component system architecture (see [Ignition Gazebo terminology](https://ignitionrobotics.org/api/gazebo/4.2/terminology.html)).
The backend will usually have multiple systems responsible for everything in the simulation – computing physics, recording logs, receiving user commands, etc.

Systems act on entities and components of those entities.
An entity is anything in the simulation scene (a model, link, light, actor, etc.), and components are their characteristics (pose, name, geometry, etc.).
Server plugins have access to all entities and what components they have, and make modifications to those components.
For example, a user could write a system that applies force to an entity by setting a component on that entity, and the physics system would pick that up and react by applying the force to make the entity actually move.
The modification of entities and components is how server plugins communicate with each other.

The entity component system is self-contained in Ignition Gazebo, and systems act on entities and their components.
Since other Ignition Libraries don't have direct knowledge/access to entities and components, other Ignition Libraries should be used in systems so that the server plugin can share entities (and their components) with these other Libraries.
For example, the physics system shares entities and components with the Ignition Physics library in order to achieve physics effects on entities and components.

There is a loop running in the backend that runs the systems, where some systems are proposing changes to entities and their components, and other systems are handling and applying those changes.
This is called the “simulation loop”.
The Entity Component Manager (ECM) in the backend provides the functionality for the actual querying and updating of the entities and components.

[Physics](https://ignitionrobotics.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1Physics.html), [User Commands](https://ignitionrobotics.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1UserCommands.html), and [Scene Broadcaster](https://ignitionrobotics.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1SceneBroadcaster.html) are all systems launched by default in the backend.
As mentioned earlier, however, even default systems can be added to or removed from the simulation loop (the [Server Configuration](https://ignitionrobotics.org/api/gazebo/4.3/server_config.html) tutorial describes how to customize default systems).
For any other functionality, like sensor data processing for example, an additional system would have to be loaded.
For example, if you need to generate sensor data that utilizes rendering sensors, you would need to load the [sensors system](https://ignitionrobotics.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1Sensors.html).
The visualization of that data, however, is left up to the client side plugins.

## Communication process

Any information that crosses the process boundary (whether back-to-front with scene broadcaster or front-to-back with user commands) has to go through [Ignition Transport](https://ignitionrobotics.org/libs/transport), the communication library.
Synchronization between processes is performed by scene broadcaster, a server-side Ignition Gazebo plugin that uses the Ignition Transport and [Ignition Messages](https://ignitionrobotics.org/libs/msgs) libraries to send messages from the server to the client.
The messages themselves are provided by Ignition Messages, while the framework for creating the publishers and subscribers that exchange messages is from Ignition Transport.

The scene broadcaster system is responsible for getting the state of the world (all of the entities and their component values) from the ever-changing simulation loop running in the back end, packaging that information into a very compact message, and periodically sending it to the frontend process.
Omitting the scene broadcaster, while possible, would mean not being able to visualize data on the frontend, which can be useful for saving computational power.

In addition to state messages sent from the scene broadcaster to the client, the client can also make requests to the server using the user commands system (another backend plugin).
It also utilizes the Ignition Transport and Ignition Messages libraries.
The user commands system has services that can be requested by the frontend GUI to insert, delete, move (etc.) models, lights, and other entities, and will send responses back to the GUI acknowledging the receipt and execution of those requests.

Several non-default plugins are able to send or receive messages to other processes outside of the server-client process.
For example, the server-side sensors plugin publishes messages outside of the server-client process, and the server-side diff-drive plugin receives info from processes other than the client.
One such process these plugins may communicate with, for example, is [ROS](https://www.ros.org/).

## Frontend client process

The client-side process, essentially the GUI, also comprises multiple plugins, some loaded by default, others that can be added, and all optional.
All of the frontend plugins use the [Ignition GUI](https://ignitionrobotics.org/libs/gui) library.
The client itself also relies on Ignition GUI.
There are visualization plugins that create the windows of the GUI, add buttons and other interactive features, and a 3D scene plugin that utilizes [Ignition Rendering](https://ignitionrobotics.org/libs/rendering) that creates the scene the user sees.

Frontend plugins typically communicate among themselves using [events](https://ignitionrobotics.org/api/gui/4.2/namespaceignition_1_1gui_1_1events.html).
Events are similar to messages, but they're processed synchronously.
For example, the `Render` event is emitted by a 3D scene from it's rendering thread right before the scene is rendered; this gives other plugins the chance to execute code right at that thread at that moment, which is valuable to edit the 3D scene.
Other such events are emitted when the user right-clicks or hovers the scene for example.

Frontend plugins all have access to the entity and component information from the compressed message provided by the scene broadcaster system from the backend.
The Entity Component Manager on the client side performs the actual message unpacking and distributes the entity and component information to the relevant plugins.
This is how visualizations know how to update in accordance with backend computations.
On the client side, the plugins are only reacting to the entity and component information, not interacting with it.

As mentioned in the previous section, the GUI can send commands back to the server using the server’s user commands system.
This would be the case, for example, if a user wanted to insert a new model through the GUI interface.
User commands receives requests from the GUI and processes them, and adds those results to entities and components in the simulation loop.
