# Use ROS 2 Simulation Interfaces to Interact with Gazebo

The [ROS 2 Simulation Interfaces](https://github.com/ros-simulation/simulation_interfaces)
define a standard set of ROS 2 service, message and action definitions for controlling and
interacting with simulation environments. These interfaces are simulator-agnostic and aim to
provide a unified way to control and observe simulation using ROS 2.

Gazebo has implemented these interfaces, enabling tasks like spawning entities, stepping
simulation, querying world state, etc. through standard ROS 2 calls. In this tutorial we
will learn how to interact with a running Gazebo simulation for the following tasks,

- [Simulation Control](#simulation-control)
- [Entity Management](#entity-management)
- [State Query](#state-query)
- [State Update](#state-update)
- [Simulator Information](#simulator-information)
- [Dependencies and Troubleshooting](#dependencies-and-troubleshooting)
- [Known Limitations](#known-limitations)

## Simulation Control

The following services and actions are available to control the flow of simulation time.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `ResetSimulation` | `/gzserver/reset_simulation` | Service | Reset the simulation to its initial state |
| `StepSimulation` | `/gzserver/step_simulation` | Service | Step the simulation forward by a specified number of steps |
| `GetSimulationState` | `/gzserver/get_simulation_state` | Service | Get the current simulation state (playing/paused/stopped) |
| `SetSimulationState` | `/gzserver/set_simulation_state` | Service | Set the simulation state (play/pause/stop) |
| `SimulateSteps` | `/gzserver/simulate_steps` | Action | Step the simulation forward by a specified number of steps with feedback and cancellation support |

### ResetSimulation Service

Reset the simulation to its initial state.

```bash
ros2 service call /gzserver/reset_simulation simulation_interfaces/srv/ResetSimulation "{}"
```

### StepSimulation Service

Step the simulation forward by a specified number of steps.

```bash
ros2 service call /gzserver/step_simulation simulation_interfaces/srv/StepSimulation "{steps: 10}"
```

### GetSimulationState Service

Get the current simulation state (playing/paused/stopped).

```bash
ros2 service call /gzserver/get_simulation_state simulation_interfaces/srv/GetSimulationState "{}"
```

### SetSimulationState Service

Set the simulation state (play/pause/stop).

- Set simulation state to stop.

  ```bash
  ros2 service call /gzserver/set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 0}}"
  ```

- Set simulation state to playing.

  ```bash
  ros2 service call /gzserver/set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"
  ```

- Set simulation state to paused.

  ```bash
  ros2 service call /gzserver/set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 2}}"
  ```

- Set simulation state to quitting.

  ```bash
  ros2 service call /gzserver/set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 3}}"
  ```

### SimulateSteps Action

Step the simulation forward by a specified number of steps with feedback and cancellation support.

```bash
ros2 action send_goal /gzserver/simulate_steps simulation_interfaces/action/SimulateSteps "{steps: 10}" --feedback
```

## Entity Management

The following interfaces are used to create or remove entities in the simulation at runtime.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `SpawnEntity` | `/gzserver/spawn_entity` | Service | Spawn a new entity in the simulation at a specific location |
| `DeleteEntity` | `/gzserver/delete_entity` | Service | Delete an existing entity by name |

### SpawnEntity Service

Spawn a new entity in the simulation at a specific location.

```bash
ros2 service call /gzserver/spawn_entity simulation_interfaces/srv/SpawnEntity "{
  name: 'my_model',
  entity_resource: {
    uri: '/path/to/model.sdf'
  },
  allow_renaming: false,
  initial_pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    }
  }
}"
```

### DeleteEntity Service

Delete an existing entity by name.

```bash
ros2 service call /gzserver/delete_entity simulation_interfaces/srv/DeleteEntity "{entity: 'my_model'}"
```

## State Query

The following interfaces are used to introspect simulation world and entity state.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `GetEntityState` | `/gzserver/get_entity_state` | Service | Get the pose and twist of a specific entity |
| `GetEntitiesStates` | `/gzserver/get_entities_states` | Service | Get the state for multiple entities (optionally filtered) |
| `GetEntities` | `/gzserver/get_entities` | Service | Get a list of entities (optionally filtered) |

### GetEntityState Service

Get the pose and twist of a specific entity.

```bash
ros2 service call /gzserver/get_entity_state simulation_interfaces/srv/GetEntityState "{entity: 'my_model'}"
```

### GetEntitiesStates Service

Get the state of multiple entities (optionally filtered).

```bash
ros2 service call /gzserver/get_entities_states simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: ''}}"
```

### GetEntites Service

Get the list of entities (optionally filtered).

```bash
ros2 service call /gzserver/get_entities simulation_interfaces/src/GetEntities "{filters: {filter: ''}}"
```

## State Update

The following interfaces are used to set entity states.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `SetEntityState` | `/gzserver/set_entity_state` | Service | Set the pose and twist of a specific entity |

### SetEntityState Service

Set the pose and twist of a specific entity.

```bash
ros2 service call /gzserver/set_entity_state simulation_interfaces/srv/SetEntityState "{ entity: 'my_model', state: {pose: { position: { x: -2.0, z: 0.5 }}, twist: {linear: {x: 0.5}}}}"
```

:::{caution}
When using `SetEntityState`, it is currently not possible to set just the `pose` or the `twist` of an entity.
Both components of the entity will be assigned based on the value of the provided message. For example, of `pose` is left empty, the pose of the entity will be set to the origin.

This might be resolved in the future. See https://github.com/ros-simulation/simulation_interfaces/issues/18
:::

## Simulator Information

Some simulators may only support a subset of interfaces. The following services can be used to inspect
supported features.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `GetSimulatorFeatures` | `/gzserver/get_simulator_features` | Service | Query which interface features are supported |

### GetSimulatorFeatures Service

Query which interface features are supported.

```bash
ros2 service call /gzserver/get_simulator_features simulation_interfaces/srv/GetSimulationFeatures "{}"
```

## Dependencies and Troubleshooting

### Required Gazebo Systems

Gazebo's ROS 2 Simulation Interfaces rely on Gazebo Transport topics and services exposed by Gazebo's world systems rather than reading simulation state directly.
In particular, [`SceneBroadcaster`](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1SceneBroadcaster.html)
(`gz::sim::systems::SceneBroadcaster`) is required to synchronize world state and detect
simulation resets.

:::{warning}
By default, `SceneBroadcaster`, `UserCommands`, and `Physics` are loaded automatically from Gazebo's server
configuration (`server.config` or `GZ_SIM_SERVER_CONFIG_PATH`).
However, if your world SDF disables server config plugins via
`<gz:policies><include_server_config_plugins>false</include_server_config_plugins></gz:policies>` (while loading
other world plugins) or if you use a custom `GZ_SIM_SERVER_CONFIG_PATH` that omits default systems, you must
explicitly include:

- `gz::sim::systems::SceneBroadcaster` — required for state synchronization and reset detection. Without it, initialization fails and state-dependent calls return `RESULT_OPERATION_FAILED` (`"Required Gazebo system is missing"`).
- `gz::sim::systems::UserCommands` — required by `SpawnEntity` and `DeleteEntity`.
- `gz::sim::systems::Physics` — required to compute dynamic entity poses and velocities for `GetEntityState` and `SetEntityState`.
:::

### Update Rates, Latency, and Timeouts

- **Nominal update rates and service latency:**
  Most `simulation_interfaces` services wait for the next state update on `SceneBroadcaster`'s `/world/<world>/state` topic before responding.
  By default, `SceneBroadcaster` throttles these updates in wall-clock time to **60 Hz** (~16.7 ms period) when simulation is running and **30 Hz** (~33.3 ms period, half of the running rate) when paused.
  Expect up to one broadcast period of latency on these calls (including `GetSimulationState` and `SetSimulationState`, which read pause/iteration status from `/world/<world>/stats` but still synchronize with `SceneBroadcaster`).

- **Event-triggered (off-rate) updates:**
  In addition to its periodic schedule, `SceneBroadcaster` immediately publishes an extra state
  message whenever simulation events occur, such as spawning or deleting entities,
  setting entity poses/twists, removing components, or resetting/rewinding time.
  During active events, the state publish rate can temporarily exceed 60 Hz / 30 Hz and service calls may unblock sooner than one nominal period.

- **Avoid setting `<state_hertz>` too low:**
  If you override `<state_hertz>` on the `SceneBroadcaster` plugin in your SDF, keep in mind that the paused rate is half of `<state_hertz>` and `simulation_interfaces` times out after **1 second** (`1000 ms`) while waiting for the next state message.
  Setting `<state_hertz>` ≤ `2.0` (when paused) or ≤ `1.0` (when running) can cause calls to fail with `"Timed out while waiting for updated Gazebo state"` whenever no simulation events occur.

- **Stepping feedback rate:**
  `StepSimulation` and the `SimulateSteps` action track completed steps from `/world/<world>/stats`, which Gazebo publishes at **10 Hz** (100 ms period) independently of `SceneBroadcaster`.

### Common Issues and Quirks

- **Brief delay querying velocities after spawning an entity:**
  Right after `SpawnEntity` succeeds, velocity tracking is enabled on the new model's canonical link via `/world/<world>/control/state`.
  Calling `GetEntityState` immediately before the server processes that update, steps physics, and broadcasts the next state can temporarily fail or report zero velocity.
  Enabling velocity tracking on all canonical links can also add overhead in worlds with many models.

- **Avoid spawning entities concurrently with `ResetSimulation`:**
  Reset completion is detected through `/world/<world>/scene/info`, which `SceneBroadcaster` also publishes whenever a new entity is added to the world.
  Spawning an entity at the same time as `ResetSimulation` can cause reset detection to trigger early.

- **Ensure the server steps on startup in custom setups:**
  `SceneBroadcaster` only advertises its state topic and service after the first simulation update step, and initial state synchronization blocks until a step runs.
  The `gzserver` node handles this automatically; if you embed the simulation interfaces alongside a custom `gz::sim::Server`, make sure the server executes an initial step or is already running.

- **Single-world and top-level model scope:**
  Only the first world reported by Gazebo (`/gazebo/worlds`) is managed.
  `GetEntities` and `GetEntitiesStates` return top-level models only (nested models are excluded), and `GetEntityState` expects a model entity.

## Known Limitations

- Only an empty string or "world" can be used in the `frame_id` field of `PoseStamped` messages. We plan to add support for using frames known to `Tf` in the future.
- Entity namespaces are not supported by the `SpawnEntity` service.
- When spawning an entity, if `SpawnEntity.allow_renaming` is set to `true` and a rename occurs in Gazebo, the new name is not returned in the `Result` object return by the `SpawnEntity` service.
