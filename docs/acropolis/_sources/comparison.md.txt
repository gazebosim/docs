**Acropolis has reached end-of-life on September 2019.
This page exists for archival purposes.
Please refer to the latest supported version.**

# Feature comparison

A list of features present in [Gazebo-classic](https://github.com/osrf/gazebo/) and
the status of their migration to [Gazebo](https://gazebosim.org/).

## Sensors

Sensor | Gazebo-classic | Gazebo Sim
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

Sensor features | Gazebo-classic | Gazebo Sim
-- | -- | --
Custom update rate | ✓ | Some sensors do, others need upgrading
Gaussian noise | ✓ | Some sensors do, others need upgrading
Custom sensors | ✓ | ✓

## Plugins

Plugin | Gazebo-classic | Gazebo Sim
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

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Play / pause / step | ✓ | ✓
Reset world / models | ✓ |
World stats | ✓ | ✓
Topic echo | ✓ | ✓
Image viewer | ✓ | ✓
Translate / rotate | ✓ |
Scale models | ✓ |
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

In Gazebo Physics, physics engines are integrated as plugins, so any engine
can be integrated without changing the core source code, as it was the case
in Gazebo.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
ODE engine | ✓ |
Bullet engine | ✓ |
DART engine | ✓ | ✓ Plugin shipped with gz-physics
Simbody engine | ✓ |
Custom engine plugins | ✕ | ✓

## Rendering

In Gazebo Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | ✓
Sky | ✓ |
Fog | ✓ |

## ROS integration

ROS integration with Gazebo will be done primarily via a
transport bridge instead of plugins, contained in the
[ros_gz](https://github.com/osrf/ros1_ign) package.

* **ROS 1**: See full message list at [ros_gz](https://github.com/osrf/ros_ign)
* **ROS 2**: Available from Blueprint

## Others

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Nested models | ✓ | Partial support
Log / playback | ✓ | Partial support
Web client (GzWeb) | ✓ |
Populations | ✓ |
Actors | ✓ |
Markers | ✓ |
Heightmaps | ✓ |
DEM (Digital Elevation Models) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
Polylines | ✓ |
Code introspection | ✓ | All simulation state is accessible from any system plugin, soon it will be published
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ |
Visual plugins | ✓ |
GUI plugins | ✓ | ✓ Gazebo GUI plugins
System plugins | ✓ |
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://github.com/osrf/gazebo_models/) | [Gazebo Fuel](https://app.gazebosim.org/fuel/models)
SDF frame semantics | | 
Saved simulation states | ✓ |
