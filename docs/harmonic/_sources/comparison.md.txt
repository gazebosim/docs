# Feature comparison

A list of features present in [Gazebo-classic](https://github.com/osrf/gazebo/)
version 11 and the status of their migration to
[Gazebo Harmonic](https://gazebosim.org/).

All the issues below are labeled with
[close the gap](https://github.com/search?q=org%3Agazebosim+label%3A%22close+the+gap%22&type=Issues)
on GitHub.

## Sensors

Sensor | Gazebo-classic | Gazebo Sim
-- | -- | --
Air pressure | ✕  | ✓
Altimeter | ✓ | ✓
Bounding Box camera | ✕ | ✓
Camera | ✓ | ✓
Contact sensor | ✓ | ✓
Depth camera | ✓ | ✓
Doppler Velocity Log (DVL) | ✕ | ✓
Force-torque | ✓ | ✓
GPS / NavSat | ✓ |  ✓
GPU Ray | ✓ | ✓ Renamed to GPU Lidar
IMU | ✓ | ✓
Logical audio sensor | ✕ | ✓
Logical camera | ✓ | ✓
Magnetometer | ✓ | ✓
Multi-camera | ✓ | ✕  Use individual cameras with same update rate
Optical tactile sensor | ✕ | ✓
Ray | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/26)
RFID sensor and tag | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/27)
RGBD camera | ✕ | ✓
Segmentation camera | ✕ | ✓
Sonar | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/19)
Thermal camera | ✕  | ✓
Triggered camera | ✕ | ✓
Wide-angle camera | ✓ | ✓
Wireless | ✓ | [Issue](https://github.com/gazebosim/gz-sensors/issues/28)

Sensor features | Gazebo-classic | Gazebo Sim
-- | -- | --
Custom update rate | ✓ | ✓
Gaussian noise | ✓ | ✓
Custom sensors | ✓ |  ✓
Laser retroreflection | ✓ | ✓
Camera distortion | ✓ | ✓
Performance metrics | ✓ |  ✓

## SDF Features

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
SDF frame semantics |✓| ✓
SDF parametrization | ✕ | [✓](http://sdformat.org/tutorials?tut=param_passing_proposal)
Load models from local files | ✓ | [✓](https://gazebosim.org/api/gazebo/6.6/resources.html)
Closed kinematic chains | ✓  | [Issue](https://github.com/gazebosim/gz-physics/issues/25)
Nested models | ✓ | ✓
Populations | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/240)
Actors | ✓ | ✓
Markers | ✓ | ✓
Heightmaps | ✓ | ✓
DEM (Digital Elevation Models) | ✓ | ✓
Polylines | ✓ | ✓
World plugins | ✓ | ✓ Now called System plugin
Model plugins | ✓ | ✓ Now called System plugin
Sensor plugins | ✓ | ✓ Now called System plugin
Visual plugins | ✓ | ✓
GUI plugins | ✓ | ✓ Gazebo GUI plugins and Gazebo GUI systems
System plugins | ✓ | ✓ Through Gazebo Launch
SDF python bindings | x | ✓ | In sdformat13
SDF <-> Mujoco MJCF | x | ✓ | In sdformat13, [documentation](https://github.com/gazebosim/gz-mujoco/blob/main/sdformat_mjcf/README.md)

## Plugins

### Model plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
AckermannSteering | ✕ | ✓
ActorPlugin | ✓ | ✕ See [FollowActor](https://github.com/gazebosim/gz-sim/blob/main/src/systems/follow_actor/FollowActor.hh) for a demo of Actor APIs
ActuatorPlugin | ✓ |
ArduCopterPlugin | ✓ |
AttachLightPlugin | ✓ | ✕ Does not apply, use SDF
Breadcrumbs | ✕ | ✓
BuoyancyPlugin | ✓ | [✓](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/buoyancy.sdf)
CartDemoPlugin | ✓ | ✕
CessnaPlugin | ✓ | ✕
DetachableJoint | ✕ | ✓
DiffDrivePlugin | ✓ | ✓
ElevatorPlugin | ✓ | ✓
FlashLightPlugin | ✓ |
FollowerPlugin | ✓ |
GimbalSmall2dPlugin | ✓ |
GravityCompensationPlugin | ✓ |
HarnessPlugin | ✓ |
HydraDemoPlugin | ✓ |
InitialVelocityPlugin | ✓ | ✓ (use VelocityControl or JointController)
JointControlPlugin | ✓ (force / pos / vel, from SDF) | ✓ (vel, from msg)
JointStatePublisher | ✕ | ✓
JointTrajectoryPlugin | ✓ | ✓
KeysToCmdVelPlugin | ✓ | Use `gz::gui::KeyPublisher` with `gz::gazebo::systems::TriggeredPublisher`
KeysToJointsPlugin | ✓ | Use `gz::gui::KeyPublisher` with `gz::gazebo::systems::TriggeredPublisher`
LedPlugin | ✓ |
LiftDragPlugin | ✓ | ✓
LinearBatteryConsumerPlugin | ✓ | ✓
LinearBatteryPlugin | ✓ | ✓
LinkPlot3DPlugin | ✓ | ✓ (renamed to Plot3D)
MecanumDrive | ✕ | ✓
MudPlugin | ✓ |
MulticopterMotorModel | ✕ | ✓
OdometryPublisherPlugin | ✕ | ✓
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
Buoyancy engine | ✕ | ✓

### World plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ArrangePlugin | ✓ |
ContainPlugin | ✓ | Partial port, [Issue](https://github.com/gazebosim/gz-sim/issues/162)
HydraPlugin | ✓ |
JoyPlugin | ✓ | ✓ Migrated as an Gazebo Launch plugin
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
ForceTorquePlugin | ✓ | ✓
GpuRayPlugin | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/49)
ImuSensorPlugin | ✓ | ✓
LensFlareSensorPlugin | ✓ |
MagnetometerPlugin | ✕ | ✓
OpticalTactilePlugin | ✕ | ✓
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
ShaderParamVisualPlugin | ✓ | ✓

### GUI plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
CessnaGUIPlugin | ✓ |
KeyboardGUIPlugin | ✓ | `gz::gui::KeyPublisher`
LookAtDemoPlugin | ✓ |
TimerGUIPlugin | ✓ |

### System plugins

Plugin | Gazebo-classic | Gazebo Sim
-- | -- | --
ColladaWorldExporter | ✕ | ✓
ModelPropShop | ✓ | [✓](https://gazebosim.org/api/gazebo/5.4/model_photo_shoot.html)
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
Insert simple lights | ✓ | ✓
Delete models | ✓ | ✓
World tree | ✓ | ✓
Scene properties | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/246)
Log recording / playback | ✓ | ✓
Plotting | ✓ | ✓
Video recording | ✓ | ✓
Screenshot | ✓ | [✓](https://gazebosim.org/api/gui/3.5/screenshot.html)
View angles | ✓ | ✓
Apply force / torque | ✓ |
Visualize as transparent | ✓ | ✓
Visualize as wireframe | ✓ | ✓
Visualize joints | ✓ |  ✓
Visualize collisions | ✓ | ✓
Visualize inertia | ✓ | ✓
Visualize CoM | ✓ |  ✓
Visualize contacts | ✓ | ✓
Visualize lights | ✓ | ✓
Follow / move to | ✓ | ✓
Copy / paste | ✓ | ✓
Building editor | ✓ |
Model editor | ✓ | [Issues](https://github.com/gazebosim/gz-sim/issues?q=is%3Aissue+is%3Aopen+label%3Aeditor)
FPS view control | ✓ |
Orthographic projection | ✓ | ✓
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

In Gazebo Physics, physics engines are integrated as plugins, so any engine
can be integrated without changing the core source code, as it was the case
in Gazebo.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
ODE engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
Bullet engine | ✓ | ✓
DART engine | ✓ | ✓ Plugin shipped with gz-physics
Simbody engine | ✓ | [Issue](https://github.com/gazebosim/gz-physics/issues/63)
TPE engine | ✕ | ✓
Custom engine plugins | ✕ | ✓
Collide bitmasks | ✓ | ✓
Restitution coefficient | ✓ | ✓
Collision detector | ✓ |  ✓
Solver | ✓ |  ✓

## Rendering

In Gazebo Rendering, render engines are integrated as plugins, so any engine
can be integrated without changing the core source code.

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Ogre 1.x engine | ✓ | ✓
Ogre 2.x engine | ✕ | ✓
Optix engine | ✕ | ✓ Partial support
Custom engine plugins | ✕ | [✓](https://gazebosim.org/api/rendering/5.0/renderingplugin.html)
Sky | ✓ | ✓
Fog | ✓ |
Material scripts | ✓ (Ogre 1.x scripts) | Does not apply
Physically Based Rendering (PBR) | ✕ | ✓ (with engines that support it, like Ogre 2)
Normal maps | ✓ | ✓
Environment maps | ✕  | ✓
Lightmaps | ✕  | ✓
Particle effects | ✕  | ✓
Render order | ✕  | ✓
Projector | ✕  | ([ogre 1.x only](https://github.com/gazebosim/gz-sim/pull/1979))

## ROS integration

ROS integration through the
[ros_gz](https://github.com/gazebosim/ros_gz) packages.

Supported versions:

* ROS 2 Iron (from source) / Rolling (from source)

Note: binaries for ROS2 might be available sometime after the Harmonic release.

For **ROS 2 Rolling**, the Rolling distribution moves with the next future release
of ROS 2 defined in [REP-2000](https://www.ros.org/reps/rep-2000.html). For the Gz
Harmonic release this means that it will be adopted when ROS 2 Jazzy goes into the
[REP-2000](https://www.ros.org/reps/rep-2000.html).

## Platforms

Platform | Gazebo-classic | Gazebo Sim
-- | -- | --
Ubuntu | ✓ | ✓
OSX | ✓ | Most of the stack works, outstanding issues: [render window](https://github.com/gazebosim/gz-sim/issues/44)
Windows | ✓ | All libraries compile, low-level libraries function well: [Issue](https://github.com/gazebosim/gz-sim/issues/168)

## Others

Feature | Gazebo-classic | Gazebo Sim
-- | -- | --
Nested models | ✓ | ✓
Log / playback | ✓ | ✓
Web client (GzWeb) | ✓ |
COLLADA meshes | ✓ | ✓
OBJ meshes | ✓ | ✓
STL meshes | ✓ | ✓
USD meshes | ✕ | [✓](https://github.com/gazebosim/sdformat/tree/sdf12/examples/usdConverter)
Code introspection | ✓ | All simulation state is accessible from system plugins or through the `SceneBroadcaster`'s state topic
Distribute simulation across processes | ✕ | (coming up)
Incrementally load levels | ✕ | ✓
Online model database | [gazebo_models repository](https://github.com/osrf/gazebo_models/) | [Gazebo Fuel](https://app.gazebosim.org/fuel/models)
Saved simulation states | ✓ | [Issue](https://github.com/gazebosim/gz-sim/issues/137)
Sphere, cylinder and box primitives | ✓ | ✓
Ellipsoid and capsule primitives | ✕ | ✓
Hydrodynamics | ✕  | ✓
Ocean currents | ✕  | ✓
Test fixture | ✓ | [✓](https://gazebosim.org/api/gazebo/6.6/test_fixture.html)
Spherical coordinates | ✓ | ✓
Generic comms system | ✕ | [✓](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/perfect_comms.sdf)
Acoustic communication | ✕ | ✓
Static linked plugins | ✕ | ✓
