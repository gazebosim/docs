# Feature comparison

A list of features present in [Gazebo-classic](https://github.com/osrf/gazebo/)
version 11 and the status of their migration to
[Ignition Edifice](https://ignitionrobotics.org/).

All the issues below are labeled with
[close the gap](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22close+the+gap%22&type=Issues)
on GitHub.

## Sensors

Sensor | Gazebo-classic | Ignition Gazebo
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Camera | ✓ | ✓
Contact sensor | ✓ | ✓
Depth camera | ✓ | ✓
Force-torque | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/25)
GPS | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/23)
GPU Ray | ✓ | ✓ Renamed to GPU Lidar
IMU | ✓ | ✓
Logical camera | ✓ | ✓
Magnetometer | ✓ | ✓
Multi-camera | ✓ | ✕  Use individual cameras with same update rate
Ray | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/26)
RFID sensor and tag | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/27)
Sonar | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/19)
Thermal camera | ✕  | ✓
Wide-angle camera | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/24)
Wireless | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/28)

Sensor features | Gazebo-classic | Ignition Gazebo
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/9)
Laser retroreflection | ✓ | ✓
Camera distortion | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/107)

## SDF Features

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
SDF frame semantics |✓| ✓
Load models from local files | ✓ | [✓](https://ignitionrobotics.org/api/gazebo/4.0/resources.html)
Closed kinematic chains | ✓  | [Issue](https://github.com/ignitionrobotics/ign-physics/issues/25)
Nested models | ✓ | ✓
Populations | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/240)
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ | ✓
DEM (Digital Elevation Models) | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/235)
Polylines | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/186)
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ | ✓ Now called System plugin
Visual plugins | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/265)
GUI plugins | ✓ | ✓ Ignition GUI plugins and Gazebo GUI systems
System plugins | ✓ | ✓ Through Ignition Launch

## Plugins

### Model plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ActorPlugin | ✓ | ✕ See [FollowActor](https://github.com/ignitionrobotics/ign-gazebo/blob/main/src/systems/follow_actor/FollowActor.hh) for a demo of Actor APIs
ActuatorPlugin | ✓ |
ArduCopterPlugin | ✓ |
AttachLightPlugin | ✓ | ✕ Does not apply, use SDF
Breadcrumbs | ✕ | ✓
BuoyancyPlugin | ✓ | [✓](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/examples/worlds/buoyancy.sdf)
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
InitialVelocityPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/50)
JointControlPlugin | ✓ (force / pos / vel, from SDF) | ✓ (vel, from msg)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ |
KeysToCmdVelPlugin | ✓ | Use `ignition::gui::KeyPublisher` with `ignition::gazebo::systems::TriggeredPublisher`
KeysToJointsPlugin | ✓ | Use `ignition::gui::KeyPublisher` with `ignition::gazebo::systems::TriggeredPublisher`
LedPlugin | ✓ |
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/231)
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
WheelTrackedVehiclePlugin | ✓ | ✓ ([partially via DiffDrivePlugin](https://github.com/ignitionrobotics/ign-gazebo/blob/44951e3ddfd238f24182d4d80b1376f0d426bd43/examples/worlds/track_drive.sdf#L2141))
KineticEnergyMonitor | ✕ | ✓

### World plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ArrangePlugin | ✓ |
ContainPlugin | ✓ | Partial port, [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/162)
HydraPlugin | ✓ |
JoyPlugin | ✓ | ✓ Migrated as an Ignition Launch plugin
MisalignmentPlugin | ✓ |
RubblePlugin | ✓ |
StaticMapPlugin | ✓ |
TransporterPlugin | ✓ |
WindPlugin | ✓ | ✓

### Sensor plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
BreakableJointPlugin | ✓ |
CameraPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
ContactPlugin | ✓ | ✓
DepthCameraPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
FiducialCameraPlugin | ✓ |
ForceTorquePlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
GpuRayPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
ImuSensorPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
LensFlareSensorPlugin | ✓ |
PressurePlugin | ✓ |
RayPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)
RaySensorNoisePlugin | ✓ | ✕ Use SDF
SonarPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/49)

### Visual plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
AmbientOcclusionVisualPlugin | ✓ |
BlinkVisualPlugin | ✓ |
HeightmapLODPlugin | ✓ |
ShaderParamVisualPlugin | ✓ |

### GUI plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
CessnaGUIPlugin | ✓ |
KeyboardGUIPlugin | ✓ | `ignition::gui::KeyPublisher`
LookAtDemoPlugin | ✓ |
TimerGUIPlugin | ✓ |

### System plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ModelPropShop | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/100)
RestUiPlugin | ✓ |
RestWebPlugin | ✓ |
StopWorldPlugin | ✓ |

## GUI

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Play / pause / step | ✓ | ✓
Reset world / models | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/203)
World stats | ✓ | ✓
Topic echo | ✓ | ✓
Image viewer | ✓ | ✓
Translate / rotate | ✓ | ✓
Scale models | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/195)
Insert models from Fuel | Partial support | ✓
Insert models from disk | ✓ | ✓
Insert simple shapes | ✓ | ✓
Insert simple lights | ✓ | ✓
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/246)
Log recording / playback | ✓ | ✓
Plotting | ✓ | ✓
Video recording | ✓ | ✓
Screenshot | ✓ | [✓](https://ignitionrobotics.org/api/gui/3.5/screenshot.html)
View angles | ✓ | ✓
Apply force / torque | ✓ |
Visualize as transparent | ✓ |
Visualize as wireframe | ✓ | ✕ (available from Fortress)
Visualize joints | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/106)
Visualize collisions | ✓ | ✓
Visualize inertia | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/111)
Visualize CoM | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/110)
Visualize contacts | ✓ | ✓
Visualize lights | ✓ | ✓
Follow / move to | ✓ | ✓
Copy / paste | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/102)
Building editor | ✓ |
Model editor | ✓ | [Issues](https://github.com/ignitionrobotics/ign-gazebo/issues?q=is%3Aissue+is%3Aopen+label%3Aeditor)
FPS view control | ✓ |
Orthographic projection | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/103)
Undo / redo | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/104)
Save world | ✓ | ✓
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓
Load GUI plugins from menu | ✕ | ✓
Edit model pose | ✓ | ✓
Edit light properties | ✓ | ✓
Edit physics properties | ✓ | ✓

## Physics

In Ignition Physics, physics engines are integrated as plugins, so any engine
can be integrated without changing the core source code, as it was the case
in Gazebo.

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
ODE engine | ✓ | [Issue](https://github.com/ignitionrobotics/ign-physics/issues/63)
Bullet engine | ✓ | [Issue](https://github.com/ignitionrobotics/ign-physics/issues/44)
DART engine | ✓ | ✓ Plugin shipped with ign-physics
Simbody engine | ✓ | [Issue](https://github.com/ignitionrobotics/ign-physics/issues/63)
TPE engine | ✕ | ✓
Custom engine plugins | ✕ | ✓
Collide bitmasks | ✓ | ✓
Restitution coefficient | ✓ | ✓
Collision detector | ✓ |  ✓
Solver | ✓ |  ✓

## Rendering

In Ignition Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | [✓](https://ignitionrobotics.org/api/rendering/5.0/renderingplugin.html)
Sky | ✓ | ✓
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x scripts) | Does not apply
Physically Based Rendering (PBR) | ✕ | ✓ (with engines that support it, like Ogre 2)
Normal maps | ✓ | ✓
Environment maps | ✕  | ✓
Lightmaps | ✕  | ✓
Particle effects | ✕  | ✓
Render order | ✕  | ✓

## ROS integration

ROS integration with Ignition will be done primarily via a
transport bridge instead of plugins, contained in the
[ros_ign](https://github.com/osrf/ros1_ign) package.

Supported versions:

* ROS 1 Melodic
* ROS 2 Dashing

## Platforms

Platform | Gazebo-classic | Ignition Gazebo
-- | -- | --
Ubuntu | ✓ | ✓
OSX | ✓ | Most of the stack works, outstanding issues: [command line](https://github.com/ignitionrobotics/ign-gazebo/issues/25), [render window](https://github.com/ignitionrobotics/ign-gazebo/issues/44)
Windows | ✓ | The stack works up to ign-gazebo: [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/168)

## Others

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Nested models | ✓ | ✓
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Code introspection | ✓ | All simulation state is accessible from system plugins or through the `SceneBroadcaster`'s state topic
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://github.com/osrf/gazebo_models/) | [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)
Saved simulation states | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/137)
Sphere, cylinder and box primitives | ✓ | ✓
Ellipsoid and capsule primitives | ✓ | ✓
