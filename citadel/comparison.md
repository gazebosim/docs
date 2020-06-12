# Feature comparison

A list of features present in [Gazebo-classic](https://bitbucket.org/osrf/gazebo/)
version 11 and the status of their migration to
[Ignition](https://ignitionrobotics.org/).

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
Distortions? | ✓ |  (barrel and pincushion distorsions not supported yet)
Custom sensors | ✓ | [Issue](https://github.com/ignitionrobotics/ign-sensors/issues/9)

## Plugins

### Model plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ActorPlugin | ✓ | TODO
ActuatorPlugin | ✓ | TODO
ArduCopterPlugin | ✓ | TODO
AttachLightPlugin | ✓ | ✕ Does not apply, use SDF
Breadcrumbs | ✕ | ✓
BuoyancyPlugin | ✓ | [issue](https://github.com/ignitionrobotics/ign-gazebo/issues/159)
CartDemoPlugin | ✓ | ✕
CessnaPlugin | ✓ | TODO
DiffDrivePlugin | ✓ | ✓
ElevatorPlugin | ✓ | TODO
FlashLightPlugin | ✓ | TODO
FollowerPlugin | ✓ | TODO
GimbalSmall2dPlugin | ✓ | TODO
GravityCompensationPlugin | ✓ | TODO
HarnessPlugin | ✓ | TODO
HydraDemoPlugin | ✓ | TODO
InitialVelocityPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/50)
JointControlPlugin | ✓ (force / pos / vel, from SDF) | ✓ (vel, from msg)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ | TODO
KeysToCmdVelPlugin | ✓ | TODO
KeysToJointsPlugin | ✓ | TODO
LedPlugin | ✓ | TODO
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ | TODO
MudPlugin | ✓ | TODO
MulticopterMotorModel | ✕ | ✓
PlaneDemoPlugin | ✓ | TODO
PosePublisher | ✕ | ✓
RandomVelocityPlugin | ✓ | TODO
RegionEventBoxPlugin | ✓ | TODO
SkidSteerDrivePlugin | ✓ | ✓
SphereAtlasDemoPlugin | ✓ | TODO
TouchPlugin | ✓ | ✓
TrackedVehiclePlugin | ✓ | TODO
VariableGearboxPlugin | ✓ | TODO
VehiclePlugin | ✓ | TODO
WheelSlipPlugin | ✓ | TODO

### World plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ArrangePlugin | ✓ | TODO
ContainPlugin | ✓ | Partial port, [issue](https://github.com/ignitionrobotics/ign-gazebo/issues/162)
HydraPlugin | ✓ | TODO
JoyPlugin | ✓ | ✓ Migrated as an Ignition Launch plugin
MisalignmentPlugin | ✓ | TODO
RubblePlugin | ✓ | TODO
StaticMapPlugin | ✓ | TODO
TransporterPlugin | ✓ | TODO
WindPlugin | ✓ | ✓

### Sensor plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
BreakableJointPlugin | ✓ | TODO
CameraPlugin | ✓ | TODO
ContactPlugin | ✓ | ✓
DepthCameraPlugin | ✓ | TODO
FiducialCameraPlugin | ✓ | TODO
ForceTorquePlugin | ✓ | TODO
GpuRayPlugin | ✓ | TODO
ImuSensorPlugin | ✓ | TODO
LensFlareSensorPlugin | ✓ | TODO
PressurePlugin | ✓ | TODO
RayPlugin | ✓ | TODO
RaySensorNoisePlugin | ✓ | TODO
SonarPlugin | ✓ | TODO

### Visual plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
AmbientOcclusionVisualPlugin | ✓ | TODO
BlinkVisualPlugin | ✓ | TODO
HeightmapLODPlugin | ✓ | TODO
ShaderParamVisualPlugin | ✓ | TODO

### GUI plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
CessnaGUIPlugin | ✓ | TODO
KeyboardGUIPlugin | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gui/issues/67)
LookAtDemoPlugin | ✓ | TODO
TimerGUIPlugin | ✓ | TODO

### System plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ModelPropShop | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/100)
RestUiPlugin | ✓ | TODO
RestWebPlugin | ✓ | TODO
StopWorldPlugin | ✓ | TODO

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
Insert models from disk | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/123)
Insert simple shapes | ✓ | ✓
Insert simple lights | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/119)
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | TODO
Log recording / playback | ✓ | ✓
Plotting | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gui/issues/66)
Video recording | ✓ | ✓
Screenshot | ✓ | TODO
View angles | ✓ | ✓
Apply force / torque | ✓ | TODO
Visualize joints | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/106)
Visualize collisions | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/105)
Visualize inertia | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/111)
Visualize CoM | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/110)
Visualize contacts | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/112)
Follow / move to | ✓ | ✓
Copy / paste | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/102)
Building editor | ✓ | TODO
Model editor | ✓ | [Design](TODO)
FPS view control | ✓ | TODO
Orthographic projection | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/103)
Undo / redo | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/104)
Save world | ✓ | ✓
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓
Load GUI plugins from menu | ✕ | ✓

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
Custom engine plugins | ✕ | [Issue](https://github.com/ignitionrobotics/ign-rendering/issues/100)

## Rendering

In Ignition Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | (coming up)
Sky | ✓ | [Issue](https://github.com/ignitionrobotics/ign-rendering/issues/98)
Fog | ✓ | TODO
Material scripts | ✓ (Ogre 1.x scripts) | Does not apply

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
Windows | ✓ | The stack works up to ign-gazebo: [issue](https://github.com/ignitionrobotics/ign-gazebo/issues/168)

## Others

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Nested models | ✓ | [SDF Issue](https://github.com/osrf/sdformat/issues/283), [Physics issue](https://github.com/ignitionrobotics/ign-physics/issues/10)
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ | TODO
Populations | ✓ | TODO
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ | TODO
DEM (Digital Elevation Models) | ✓ | TODO
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Polylines | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/186)
Code introspection | ✓ | All simulation state is accessible from system plugins or through the `SceneBroadcaster`'s state topic
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ | ✓ Now called System plugin
Visual plugins | ✓ | TODO
GUI plugins | ✓ | ✓ Ignition GUI plugins and Gazebo GUI systems
System plugins | ✓ | ✓ Through Ignition Launch
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://bitbucket.org/osrf/gazebo_models/) | [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)
SDF frame semantics |✓ | ✓
Saved simulation states | ✓ | [Issue](https://github.com/ignitionrobotics/ign-gazebo/issues/137)
