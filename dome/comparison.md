# Feature comparison

A list of features present in [Gazebo-classic](https://github.com/osrf/gazebo/)
version 11 and the status of their migration to
[Ignition Dome](https://gazebosim.org/).

All the issues below are labeled with
[close the gap](https://github.com/search?q=org%3Aignitionrobotics+label%3A%22close+the+gap%22&type=Issues)
on GitHub.

## Sensors

Sensor | Gazebo-classic | Gazebo Sim
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Bounding Box camera | ✕ | ✕  (available from Fortress)
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
Segmentation camera | ✕ | ✕  (available from Fortress)
Sonar | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/19)
Thermal camera | ✕  | ✓
Wide-angle camera | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/24)
Wireless | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/28)

Sensor features | Gazebo-classic | Gazebo Sim
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ |  ✕  (available from Fortress)
Laser retroreflection | ✓ | ✓
Camera distortion | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/107)
Performance metrics | ✓ |  ✓

## SDF Features

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
SDF frame semantics |✓| ✓
SDF parametrization | ✕ | [✓](http://sdformat.org/tutorials?tut=param_passing_proposal)
Load models from local files | ✓ | [✓](https://gazebosim.org/api/gazebo/4.0/resources.html)
Closed kinematic chains | ✓  | [Issue](https://github.com/gazebosim/gz-physics/issues/25)
Nested models | ✓ | Partial support, fully available from Edifice
Populations | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/240)
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ | ✕ (Ogre 1 from Edifice, Ogre 2 from Fortress)
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
BuoyancyPlugin | ✓ | [✓](https://github.com/gazebosim/gz-sim/blob/ign-gazebo4/examples/worlds/buoyancy.sdf)
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
SimpleTrackedVehiclePlugin | ✓ | ✓
SkidSteerDrivePlugin | ✓ | ✓
SphereAtlasDemoPlugin | ✓ | ✕
TouchPlugin | ✓ | ✓
TrackedVehiclePlugin | ✓ | ✓
VariableGearboxPlugin | ✓ |
VehiclePlugin | ✓ |
WheelSlipPlugin | ✓ | ✓
WheelTrackedVehiclePlugin | ✓ | ✓
KineticEnergyMonitor | ✕ | ✓
Buoyancy engine | ✓ | ✕  (available from Fortress)

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
Insert simple lights | ✓ | ✕  (available from Edifice)
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/246)
Log recording / playback | ✓ | ✓
Plotting | ✓ | ✓
Video recording | ✓ | ✓
Screenshot | ✓ | [✓](https://gazebosim.org/api/gui/3.5/screenshot.html)
View angles | ✓ | ✓
Apply force / torque | ✓ |
Visualize as transparent | ✓ | ✕ (available from Fortress)
Visualize as wireframe | ✓ | ✕ (available from Fortress)
Visualize joints | ✓ |  ✕ (available from Fortress)
Visualize collisions | ✓ | ✓
Visualize inertia | ✓ | ✕ (available from Fortress)
Visualize CoM | ✓ |  ✕ (available from Fortress)
Visualize contacts | ✓ | ✓
Visualize lights | ✓ | ✕  (available from Edifice)
Follow / move to | ✓ | ✓
Copy / paste | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/102)
Building editor | ✓ |
Model editor | ✓ | [Issues](https://github.com/gazebosim/gz-sim/issues?q=is%3Aissue+is%3Aopen+label%3Aeditor)
FPS view control | ✓ |
Orthographic projection | ✓ | ✕ (available from Fortress)
Undo / redo | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/104)
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

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
ODE engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
Bullet engine | ✓ | ✓
DART engine | ✓ | ✓ Plugin shipped with ign-physics
Simbody engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
TPE engine | ✕ | ✓
Custom engine plugins | ✕ | ✓
Collide bitmasks | ✓ | ✓
Restitution coefficient | ✓ | ✓
Collision detector | ✓ |  ✕  (available from Edifice)
Solver | ✓ |  ✕  (available from Edifice)

## Rendering

In Ignition Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | [✓](https://gazebosim.org/api/rendering/4.7/renderingplugin.html)
Sky | ✓ | ✕  (available from Edifice)
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x scripts) | Does not apply
Physically Based Rendering (PBR) | ✕ | ✓ (with engines that support it, like Ogre 2)
Normal maps | ✓ | ✓
Environment maps | ✕  | ✓
Lightmaps | ✕  | ✕  (available from Edifice)
Particle effects | ✕  | ✓
Render order | ✕  | ✕  (available from Edifice)

## ROS integration

ROS integration through the
[ros_ign](https://github.com/ignitionrobotics/ros_ign) packages.

Supported versions:

* ROS 1 Melodic (binaries) / Noetic (from source)
* ROS 2 Foxy (from source)

## Platforms

Platform | Gazebo-classic | Gazebo Sim
-- | -- | --
Ubuntu | ✓ | ✓
OSX | ✓ | Most of the stack works, outstanding issues: [command line](https://github.com/gazebosim/gz-sim/issues/25), [render window](https://github.com/gazebosim/gz-sim/issues/44)
Windows | ✓ | The stack works up to ign-gazebo: [Issue](https://github.com/gazebosim/gz-sim/issues/168)

## Others

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Nested models | ✓ | ✕  (available from Edifice)
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
Sphere, cylinder and box primitives | ✓ | ✓
Ellipsoid and capsule primitives | ✕ | ✕  (available from Edifice)
Hydrodynamics | ✕  | ✕  (available from Edifice)
Ocean currents | ✕  | ✕  (available from Edifice)
Test fixture | ✓ | [✓](https://gazebosim.org/api/gazebo/4.9/test_fixture.html)
Spherical coordinates | ✓ | ✕ (available from Fortress)
