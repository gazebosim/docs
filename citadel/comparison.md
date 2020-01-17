# Feature comparison

A list of features present in [Gazebo-classic](https://bitbucket.org/osrf/gazebo/) and
the status of their migration to [Ignition](https://ignitionrobotics.org/).

## Sensors

Sensor | Gazebo-classic | Ignition Gazebo
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Camera | ✓ | ✓
Contact sensor | ✓ | ✓
Depth camera | ✓ | ✓
Force-torque | ✓ |
GPS | ✓ |
GPU Ray | ✓ | ✓ Renamed to GPU Lidar
IMU | ✓ | ✓
Logical camera | ✓ | ✓
Magnetometer | ✓ | ✓
Multi-camera | ✓ |
Ray | ✓ |
RFID | ✓ |
RFIDTag | ✓ |
Sonar | ✓ |
Thermal camera | ✕  | ✓
Wide-angle camera | ✓ |
Wireless receiver | ✓ |
Wireless transceiver | ✓ |
Wireless transmitter | ✓ |

Sensor features | Gazebo-classic | Ignition Gazebo
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ | ✓ (barrel and pincushion distorsions not supported yet)

## Plugins

Plugin | Gazebo-classic | Ignition Gazebo
-- | -- | --
ActorPlugin | ✓ |
ActuatorPlugin | ✓ |
AmbientOcclusionVisualPlugin | ✓ |
ArduCopterPlugin | ✓ |
ArrangePlugin | ✓ |
AttachLightPlugin | ✓ | ✕ Does not apply, use SDF
BlinkVisualPlugin | ✓ |
Breadcrumbs | ✕ | ✓
BreakableJointPlugin | ✓ |
BuoyancyPlugin | ✓ |
CameraPlugin | ✓ |
CartDemoPlugin | ✓ |
CessnaPlugin | ✓ |
ContactPlugin | ✓ | ✓
ContainPlugin | ✓ |
DepthCameraPlugin | ✓ |
DiffDrivePlugin | ✓ | ✓
FiducialCameraPlugin | ✓ |
FlashLightPlugin | ✓ |
FollowerPlugin | ✓ |
ForceTorquePlugin | ✓ |
GimbalSmall2dPlugin | ✓ |
GpuRayPlugin | ✓ |
HarnessPlugin | ✓ |
HeightmapLODPlugin | ✓ |
ImuSensorPlugin | ✓ |
InitialVelocityPlugin | ✓ |
JointControlPlugin | ✓ (force / pos / vel, from SDF) | ✓ (vel, from msg)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ |
KeysToJointsPlugin | ✓ |
LedPlugin | ✓ |
LensFlareSensorPlugin | ✓ |
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ |
ModelPropShop | ✓ |
MudPlugin | ✓ |
MulticopterMotorModel | ✕ | ✓
PlaneDemoPlugin | ✓ |
PosePublisher | ✕ | ✓
PressurePlugin | ✓ |
RayPlugin | ✓ | Provided through Ignition Sensors
RaySensorNoisePlugin | ✓ |
RubblePlugin | ✓ |
ShaderParamVisualPlugin | ✓ |
SkidSteerDrivePlugin | ✓ | ✓
SonarPlugin | ✓ |
SphereAtlasDemoPlugin | ✓ |
StaticMapPlugin | ✓ |
StopWorldPlugin | ✓ |
TouchPlugin | ✓ | ✓
VehiclePlugin | ✓ |
WheelSlipPlugin | ✓ |
WindPlugin | ✓ | ✓
ElevatorPlugin | ✓ |
RandomVelocityPlugin | ✓ |
TransporterPlugin | ✓ |
HydraPlugin | ✓ |
HydraDemoPlugin | ✓ |
JoyPlugin | ✓ | ✓ Migrated as an Ignition Launch plugin
CessnaGUIPlugin | ✓ |
KeyboardGUIPlugin | ✓ |
LookAtDemoPlugin | ✓ |
TimerGUIPlugin | ✓ |
GravityCompensationPlugin | ✓ |

## GUI

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Play / pause / step | ✓ | ✓
Reset world / models | ✓ |
World stats | ✓ | ✓
Topic echo | ✓ | ✓
Image viewer | ✓ | ✓
Translate / rotate | ✓ | ✓
Scale models | ✓ |
Insert models / lights | ✓ | ✓ (dragging Fuel URL)
Delete models | ✓ | ✓
World tree | ✓ | ✓ (no properties yet)
Log recording / playback | ✓ | ✓
Plotting | ✓ |
Video recording | ✓ | ✓
Screenshot | ✓ |
View angles | ✓ |
Apply force / torque | ✓ |
Introspection visualizations (transparent, joints...) | ✓ |
Follow / move to | ✓ | ✓
Copy / paste | ✓ |
Building editor | ✓ |
Model editor | ✓ |
FPS view control | ✓ |
Orthographic projection | ✓ |
Save world | ✓ |
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓
Load plugins from menu | ✕ | ✓

## Physics

In Ignition Physics, physics engines are integrated as plugins, so any engine
can be integrated without changing the core source code, as it was the case
in Gazebo.

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
ODE engine | ✓ |
Bullet engine | ✓ |
DART engine | ✓ | ✓ Plugin shipped with ign-physics
Simbody engine | ✓ |
Custom engine plugins | ✕ | ✓

## Rendering

In Ignition Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | ✓
Sky | ✓ |
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x scripts) | ✕

## ROS integration

ROS integration with Ignition will be done primarily via a
transport bridge instead of plugins, contained in the
[ros_ign](https://github.com/osrf/ros1_ign) package.

Supported versions:

* ROS 1 Melodic
* ROS 2 Dashing

## Others

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Nested models | ✓ | Partial support
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ |
Populations | ✓ |
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ |
DEM (Digital Elevation Models) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Polylines | ✓ |
Code introspection | ✓ | All simulation state is accessible from system plugins or through the `SceneBroadcaster`'s state topic
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ | ✓ Now called System plugin
Visual plugins | ✓ |
GUI plugins | ✓ | ✓ Ignition GUI plugins and Gazebo GUI systems
System plugins | ✓ | ✓ Through Ignition Launch
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://bitbucket.org/osrf/gazebo_models/) | [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)
SDF frame semantics | | ✓
