# Feature comparison

A list of features present in [Gazebo-classic](https://github.com/osrf/gazebo/)
version 11 and the status of their migration to
[Ignition Blueprint](https://gazebosim.org/).

All the issues below are labeled with
[close the gap](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22close+the+gap%22&type=Issues)
on GitHub.

## Sensors

Sensor | Gazebo-classic | Gazebo Sim
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Camera | ✓ | ✓
Contact sensor | ✓ | ✓
Depth camera | ✓ | ✓
Force-torque | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/25)
GPS | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/23)
GPU Ray | ✓ | ✓ Renamed to GPU Lidar
IMU | ✓ | ✓
Logical camera | ✓ | ✓
Magnetometer | ✓ | ✓
Multi-camera | ✓ | ✕  Use individual cameras with same update rate
Ray | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/26)
RFID sensor and tag | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/27)
Sonar | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/19)
Thermal camera | ✕  | ✕ (available from Citadel)
Wide-angle camera | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/24)
Wireless | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/28)

Sensor features | Gazebo-classic | Gazebo Sim
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/9)

## SDF Features

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
SDF frame semantics |✓| ✕ (available from Citadel)
Load models from local files | ✓ | [✓](https://gazebosim.org/api/gazebo/2.24/resources.html)
Closed kinematic chains | ✓  | [Issue](https://github.com/gazebosim/gz-physics/issues/25)
Nested models | ✓ | Partial support
Populations | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/240)
Actors | ✓ |  ✕ (available from Citadel)
Markers | ✓ |  ✕ (available from Citadel)
Heightmaps | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/237)
DEM (Digital Elevation Models) | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/235)
Polylines | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/186)
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ | ✓ Now called System plugin
Visual plugins | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/265)
GUI plugins | ✓ | ✓ Ignition GUI plugins and Gazebo GUI systems
System plugins | ✓ | ✓ Through Ignition Launch

## Plugins

### Model plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ActorPlugin | ✓ | ✕ See [FollowActor](https://github.com/gazebosim/gz-sim/blob/main/src/systems/follow_actor/FollowActor.hh) for a demo of Actor APIs
ActuatorPlugin | ✓ |
ArduCopterPlugin | ✓ |
AttachLightPlugin | ✓ | ✕ Does not apply, use SDF
Breadcrumbs | ✕ | ✓
BuoyancyPlugin | ✓ | ✕ (available from Citadel)
CartDemoPlugin | ✓ | ✕
CessnaPlugin | ✓ |
DiffDrivePlugin | ✓ | ✓
ElevatorPlugin | ✓ |
FlashLightPlugin | ✓ |
FollowerPlugin | ✓ |
GimbalSmall2dPlugin | ✓ |
GravityCompensationPlugin | ✓ |
HarnessPlugin | ✓ |
HydraDemoPlugin | ✓ |
InitialVelocityPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/50)
JointControlPlugin | ✓ (force / pos / vel, from SDF) | ✓ (vel, from msg)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ |
KeysToCmdVelPlugin | ✓ | Use `ignition::gui::KeyPublisher` with `ignition::gazebo::systems::TriggeredPublisher`
KeysToJointsPlugin | ✓ | Use `ignition::gui::KeyPublisher` with `ignition::gazebo::systems::TriggeredPublisher`
LedPlugin | ✓ |
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/231)
MudPlugin | ✓ |
MulticopterMotorModel | ✕ | ✓
PlaneDemoPlugin | ✓ |
PosePublisher | ✕ | ✓
RandomVelocityPlugin | ✓ |
RegionEventBoxPlugin | ✓ |
SimpleTrackedVehiclePlugin | ✓ |
SkidSteerDrivePlugin | ✓ | ✓
SphereAtlasDemoPlugin | ✓ | ✕
TouchPlugin | ✓ | ✓
TrackedVehiclePlugin | ✓ |
VariableGearboxPlugin | ✓ |
VehiclePlugin | ✓ |
WheelSlipPlugin | ✓ | ✓
WheelTrackedVehiclePlugin | ✓ | ✓ ([partially via DiffDrivePlugin](https://github.com/gazebosim/gz-sim/blob/44951e3ddfd238f24182d4d80b1376f0d426bd43/examples/worlds/track_drive.sdf#L2141))

### World plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ArrangePlugin | ✓ |
ContainPlugin | ✓ | Partial port, [Issue](https://github.com/gazebosim/gz-sim/issues/162)
HydraPlugin | ✓ |
JoyPlugin | ✓ | ✓ Migrated as an Ignition Launch plugin
MisalignmentPlugin | ✓ |
RubblePlugin | ✓ |
StaticMapPlugin | ✓ |
TransporterPlugin | ✓ |
WindPlugin | ✓ | ✓

### Sensor plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
BreakableJointPlugin | ✓ |
CameraPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
ContactPlugin | ✓ | ✓
DepthCameraPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
FiducialCameraPlugin | ✓ |
ForceTorquePlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
GpuRayPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
ImuSensorPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
LensFlareSensorPlugin | ✓ |
PressurePlugin | ✓ |
RayPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
RaySensorNoisePlugin | ✓ | ✕ Use SDF
SonarPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)

### Visual plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
AmbientOcclusionVisualPlugin | ✓ |
BlinkVisualPlugin | ✓ |
HeightmapLODPlugin | ✓ |
ShaderParamVisualPlugin | ✓ |

### GUI plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
CessnaGUIPlugin | ✓ |
KeyboardGUIPlugin | ✓ | `ignition::gui::KeyPublisher`
LookAtDemoPlugin | ✓ |
TimerGUIPlugin | ✓ |

### System plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ModelPropShop | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/100)
RestUiPlugin | ✓ |
RestWebPlugin | ✓ |
StopWorldPlugin | ✓ |

## GUI

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Play / pause / step | ✓ | ✓
Reset world / models | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/203)
World stats | ✓ | ✓
Topic echo | ✓ | ✓
Image viewer | ✓ | ✓
Translate / rotate | ✓ | ✓
Scale models | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/195)
Insert models from Fuel | Partial support | ✓
Insert models from disk | ✓ | ✓
Insert simple shapes | ✓ | ✓
Insert simple lights | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/119)
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/246)
Log recording / playback | ✓ | ✓
Plotting | ✓ | ✕ (available from Dome)
Video recording | ✓ | ✓
Screenshot | ✓ | [Issue](https://github.com/gazebosim/gz-gui/issues/95)
View angles | ✓ | ✓
Apply force / torque | ✓ |
Visualize as transparent | ✓ |
Visualize as wireframe | ✓ |
Visualize joints | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/106)
Visualize collisions | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/105)
Visualize inertia | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/111)
Visualize CoM | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/110)
Visualize contacts | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/112)
Follow / move to | ✓ | ✓
Copy / paste | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/102)
Building editor | ✓ |
Model editor | ✓ | [Issues](https://github.com/gazebosim/gz-sim/issues?q=is%3Aissue+is%3Aopen+label%3Aeditor)
FPS view control | ✓ |
Orthographic projection | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/103)
Undo / redo | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/104)
Save world | ✓ | ✓
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓
Load GUI plugins from menu | ✕ | ✓

## Physics

In Ignition Physics, physics engines are integrated as plugins, so any engine
can be integrated without changing the core source code, as it was the case
in Gazebo.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
ODE engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
Bullet engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/44)
DART engine | ✓ | ✓ Plugin shipped with ign-physics
Simbody engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
TPE engine | ✕ | ✕ (available from Citadel)
Custom engine plugins | ✕ | ✓

## Rendering

In Ignition Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | [Issue](https://github.com/gazebosim/gz-rendering/issues/100)
Sky | ✓ | [Issue](https://github.com/gazebosim/gz-rendering/issues/98)
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x scripts) | Does not apply

## ROS integration

ROS integration with Ignition will be done primarily via a
transport bridge instead of plugins, contained in the
[ros_ign](https://github.com/osrf/ros1_ign) package.

Supported versions:

* ROS 1 Melodic
* ROS 2 Dashing

## Platforms

Platform | Gazebo-classic | Gazebo Sim
-- | -- | --
Ubuntu | ✓ | ✓
OSX | ✓ | Most of the stack works, outstanding issues: [command line](https://github.com/gazebosim/gz-sim/issues/25), [render window](https://github.com/gazebosim/gz-sim/issues/44)
Windows | ✓ | The stack works up to ign-gazebo: [Issue](https://github.com/gazebosim/gz-sim/issues/168)

## Others

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Nested models | ✓ | [Physics issue](https://github.com/gazebosim/gz-physics/issues/10)
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Code introspection | ✓ | All simulation state is accessible from system plugins or through the `SceneBroadcaster`'s state topic
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://github.com/osrf/gazebo_models/) | [Ignition Fuel](https://app.gazebosim.org/fuel/models)
Saved simulation states | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/137)
