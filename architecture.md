# Ignition Gazebo Architecture

Ignition Gazebo is the application entry-point of Ignition architecture, generally encompassing the use of all other Ignition libraries.
It is a library itself, but its use case described in this document is as an executable.
As an executable, it runs a simulation by launching two processes: the backend server process and frontend client process.

The core of Ignition Gazebo is the server-client communication and the various plugins it loads to facilitate the simulation.
Plugins are loaded at runtime to create a link between other Ignition libraries and Ignition Gazebo so they can exchange information.
Users can write their own plugins.
Because they’re loaded at runtime, Ignition does not need to be recompiled to add or remove plugins.
There are other types of plugins in the scope of Ignition; however, this use case only concerns Ignition Gazebo server and GUI plugins.

Without plugins, Ignition libraries are not “aware” of each other and don’t possess the ability to interact.
For example, Ignition Physics is independent of Ignition Gazebo, and vice versa.
So Ignition Gazebo has a physics plugin that uses Ignition Physics as a library and incorporates Gazebo specifics, to enable Physics to be a system running in the simulation loop.
This is the case for all libraries, whether their corresponding plugins affect the frontend or backend processes.

The Ignition Physics library is only used in the physics plugin, not in other plugins nor in the core of Ignition Gazebo.
Some libraries only have one corresponding plugin, or one in each process (frontend and backend).
However, some libraries are used by all plugins, like Ignition Common which provides common functionality like logging, manipulating strings, file system interaction, etc., to all plugins.
Other such libraries include Ignition Plugin, Ignition Math, SDFormat, and more.

There are certain plugins in both the frontend and backend processes that are loaded by default in every version of Ignition.
Many other plugins are optional.
One common optional plugin is the sensors plugin.
Both optional and default plugins can be removed or added at any time; Ignition Gazebo will continue to run with limited functionality.

![Ignition Gazebo architecture diagram]()

## Backend server process

Ignition gazebo is responsible for loading plugins in the backend, referred to as systems.
The server runs an entity-component system architecture (see [Ignition Gazebo terminology](https://ignitionrobotics.org/api/gazebo/4.2/terminology.html)).
The backend will usually have multiple systems responsible for everything in the simulation – computing physics, recording logs, receiving user commands, etc.

Systems act on entities and components of those entities.
An entity is anything in the simulation scene (a model, link, light, actor, etc.), and components are their characteristics (pose, name, geometry, etc.).
Server plugins have access to all entities and what components they have, and make modifications to those components.
For example, a user could write a system that applies force to an entity by setting a component on that entity, and the physics system would pick that up and react by applying the force to make the entity actually move.
The modification of entities and components is how server plugins communicate with each other.
Libraries, however, don’t know about the entities and components.

All of the systems in the backend create a loop, where some systems are proposing changes to entities and their components, and other systems are handling and applying those changes.
This is called the “Simulation loop”.

Physics, user commands, and scene broadcaster are all systems launched by default in the backend.
As mentioned earlier, however, even default systems can be added to or removed from this loop.
For any other functionality, like sensor data processing for example, a new system would have to be added.
Plugins that generate sensor data and utilize the Ignition Sensors and Ignition Rendering libraries would have to be added to the backend process.
The visualization of that data, however, is left up to the client side plugins.

## Communication process

Synchronization between processes is performed by scene broadcaster, a server-side Ignition Gazebo plugin that uses Ignition Transport and Ignition Messages libraries to send messages from the server to the client.
The messages themselves are provided by Ignition Messages, while the framework for creating the publishers and subscribers that exchange messages is from Ignition Transport.

The scene broadcaster system is responsible for getting the state of the world (all of the entities and their component values) from the ever-changing simulation loop running in the back end, packaging that information into a very compact message, and periodically sending it to the frontend to process.
Omitting this system would mean not being able to visualize data on the frontend, which can be useful for saving computational power.

In addition to state messages sent from the scene broadcaster to the client, the client can also make requests to the server using the user commands system (another backend plugin).
It also utilizes the Ignition Transport and Ignition Messages libraries.
The user command system has services that can be requested by the frontend GUI to insert, delete, move, etc., and will send responses back to the GUI acknowledging the receipt and execution of those requests.


## Frontend client process

The client-side process, essentially the GUI, also comprises multiple plugins, some loaded by default, others that can be added, and all optional.
All of the frontend plugins use the Ignition GUI library.
There are visualization plugins that create the windows of the GUI, add buttons and other interactive features, and a 3D scene plugin that utilizes Ignition Rendering that creates the scene the user sees.

Frontend plugins communicate using events.
They all have access to the entity and component information from the compressed message provided by the scene broadcaster system from the backend.
This is how visualizations know how to update in accordance with backend computations.

As mentioned in the previous section, the GUI can send commands back to the server using the server’s user commands system.
This would be the case, for example, if a user wanted to insert a new model through the GUI interface.
User commands receives requests from the GUI and processes them, and adds those results to entities and components in the simulation loop.
Any information that crosses the process boundary (whether back-to-front with scene broadcaster or front-to-back with user commands) has to go through Ignition Transport, the communication library.
