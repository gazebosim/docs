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
- [Simulator Information](#simulator-information)

## Simulation Control

The following services and actions are available to control the flow of simulation time.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `ResetSimulation` | `/reset_simulation` | Service | Reset the simulation to its initial state |
| `StepSimulation` | `/step_simulation` | Service | Step the simulation forward by a specified number of steps |
| `GetSimulationState` | `/get_simulation_state` | Service | Get the current simulation state (playing/paused/stopped) |
| `SetSimulationState` | `/set_simulation_state` | Service | Set the simulation state (play/pause/stop) |
| `SimulateSteps` | `/simulate_steps` | Action | Step the simulation forward by a specified number of steps with feedback and cancellation support | 

**ResetSimulation Service**

Reset the simulation to its initial state.

```bash
ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{}"
```

**StepSimulation Service**

Step the simulation forward by a specified number of steps.

```bash
ros2 service call /step_simulation simulation_interfaces/srv/StepSimulation "{steps: 10}"
```

**GetSimulationState Service**

Get the current simulation state (playing/paused/stopped).

```bash
ros2 service call /get_simulation_state simulation_interfaces/srv/GetSimulationState "{}"
```

**SetSimulationState Service**

Set the simulation state (play/pause/stop).

- Set simulation state to stop.

  ```bash
  ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 0}}"
  ```

- Set simulation state to playing.

  ```bash
  ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"
  ```

- Set simulation state to paused.

  ```bash
  ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 2}}"
  ```

- Set simulation state to quitting.

  ```bash
  ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{state: {state: 3}}"
  ```

**SimulateSteps Action**

Step the simulation forward by a specified number of steps with feedback and cancellation support.

```bash
ros2 action send_goal /simulate_steps simulation_interfaces/action/SimulateSteps "{steps: 10}" --feedback
```

## Entity Management

The following interfaces are used to create or remove entities in the simulation at runtime.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `SpawnEntity` | `/spawn_entity` | Service | Spawn a new entity in the simulation at a specific location |
| `DeleteEntity` | `/delete_entity` | Service | Delete an existing entity by name |

**SpawnEntity Service**

Spawn a new entity in the simulation at a specific location.

```bash
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{
  name: 'my_model',
  uri: '/path/to/model.sdf',
  allow_renaming: false,
  initial_pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    }
  }
}"
```

**DeleteEntity Service**

Delete an existing entity by name.

```bash
ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{entity: 'my_model'}"
```

## State Query

The following interfaces are used to introspect simulation world and entity state.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `GetEntityState` | `/get_entity_state` | Service | Get the pose and twist of a specific entity |
| `GetEntitiesStates` | `/get_entities_states` | Service | Get the state for multiple entities (optionally filtered) |
| `GetEntities` | `/get_entities` | Service | Get a list of entities (optionally filtered) |

**GetEntityState Service**

Get the pose and twist of a specific entity.

```bash
ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState "{entity: 'my_model'}"
```

**GetEntitiesStates Service**

Get the state of multiple entities (optionally filtered).

```bash
ros2 service call /get_entities_states simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: ''}}"
```

**GetEntites Service**

Get the list of entities (optionally filtered).

```bash
ros2 service call /get_entities simulation_interfaces/src/GetEntities "{filters: {filter: ''}}"
```

## Simulator Information

Some simulators may only support a subset of interfaces. The following services can be used to inspect
supported features.

| Interface Name | Topic Name | Type | Description |
|----------------|------------|------|-------------|
| `GetSimulatorFeatures` | `/get_simulator_features` | Service | Query which interface features are supported |

**GetSimulatorFeatures Service**

Query which interface features are supported.

```bash
ros2 service call /get_simulator_features simulation_interfaces/srv/GetSimulationFeatures "{}"
```
