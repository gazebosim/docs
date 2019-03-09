# Feature comparison

A list of features present in [Gazebo-classic](https://bitbucket.org/osrf/gazebo/) and
the status of their migration to [Ignition](https://ignitionrobotics.org/).

## Sensors

Sensor | Gazebo-classic | Ignition Gazebo
-- | -- | --
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
Wide-angle camera | ✓ |
Wireless receiver | ✓ |
Wireless transceiver | ✓ |
Wireless transmitter | ✓ |

Sensor features | Gazebo-classic | Ignition Gazebo
-- | -- | --
Custom update rate | ✓ | Some sensors do, others need upgrading
Gaussian noise | ✓ | Some sensors do, others need upgrading
Custom sensors | ✓ | ✓

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
BreakableJointPlugin | ✓ |
BuoyancyPlugin | ✓ |
CameraPlugin | ✓ |
CartDemoPlugin | ✓ |
CessnaPlugin | ✓ |
ContactPlugin | ✓ |
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
JointControlPlugin | ✓ |
JointTrajectoryPlugin | ✓ |
KeysToJointsPlugin | ✓ |
LedPlugin | ✓ |
LensFlareSensorPlugin | ✓ |
LiftDragPlugin | ✓ |
LinearBatteryConsumerPlugin | ✓ |
LinearBatteryPlugin | ✓ |
LinkPlot3DPlugin | ✓ |
ModelPropShop | ✓ |
MudPlugin | ✓ |
PlaneDemoPlugin | ✓ |
PressurePlugin | ✓ |
RayPlugin | ✓ |
RaySensorNoisePlugin | ✓ |
RubblePlugin | ✓ |
ShaderParamVisualPlugin | ✓ |
SkidSteerDrivePlugin | ✓ |
SonarPlugin | ✓ |
SphereAtlasDemoPlugin | ✓ |
StaticMapPlugin | ✓ |
StopWorldPlugin | ✓ |
TouchPlugin | ✓ | ✓
VehiclePlugin | ✓ |
WheelSlipPlugin | ✓ |
WindPlugin | ✓ |
ElevatorPlugin | ✓ |
RandomVelocityPlugin | ✓ |
TransporterPlugin | ✓ |
HydraPlugin | ✓ |
HydraDemoPlugin | ✓ |
JoyPlugin | ✓ | ✓ Migrated as standalone program
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
Translate / rotate / scale models | ✓ |
Insert models / lights | ✓ |
Delete models | ✓ |
World tree | ✓ |
Log recording / playback | ✓ |
Plotting | ✓ |
Video recording | ✓ |
Screenshot | ✓ |
View angles | ✓ |
Apply force / torque | ✓ |
Introspection visualizations (transparent, joints...) | ✓ |
Follow / move to | ✓ |
Copy / paste | ✓ |
Building editor | ✓ |
Model editor | ✓ |
FPS view control | ✓ |
Orthographic projection | ✓ |
Save world | ✓ |
Save GUI configuration | ✓ | ✓
Color scheme and themes | ✕ | ✓
Position, resize and configure widgets | ✕ | ✓

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

## ROS integration

ROS integration with Ignition will be done primarily via a
transport bridge instead of plugins.

* **ROS 1**: See full message list at [ros1_ign_bridge](https://github.com/osrf/ros1_ign_bridge)
* **ROS 2**: TODO

## Others

Feature | Gazebo-classic | Ignition Gazebo
-- | -- | --
Nested models | ✓ | Partial support
Log / playback | ✓ | Partial support
Web client (GzWeb) | ✓ |
Populations | ✓ |
Actors | ✓ |
Heightmaps | ✓ |
DEM (Digital Elevation Models) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Code introspection | ✓ | All simulation state is accessible from any system plugin, soon it will be published
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ |
Visual plugins | ✓ |
GUI plugins | ✓ | ✓ Ignition GUI plugins
System plugins | ✓ |
Distribute simulation across processes | ✕ | ✓
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://bitbucket.org/osrf/gazebo_models/) | [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)

