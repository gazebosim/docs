# Performance tuning

If your Jetty simulation is running below real time or the GUI feels slow, start
by identifying which part of the workload is expensive. A world with many
contacts is tuned differently from a world dominated by cameras, lidar, or
rendering.

## Measure first

Before changing settings, enable the world statistics tools described in
[SDF Worlds](sdf_worlds). In particular, keep an eye on:

* real time factor (RTF)
* simulation time versus real time
* whether the slowdown appears when physics, sensors, or rendering load changes

This gives you a baseline before you start trading fidelity for speed.

## Common performance tweaks

### Reduce physics work

* Try less aggressive physics settings in [SDF Worlds](sdf_worlds). Larger
  `<max_step_size>` values usually reduce computation, but they also reduce
  accuracy and can change contact behavior.
* Use simpler collision geometry when possible. [Building your own robot](building_robot)
  notes that collision geometry does not need to match the visual mesh exactly,
  and simpler collisions often reduce computation time.

### Reduce sensor work

* Lower `<update_rate>` for sensors that do not need high-frequency data. See
  [Sensors](sensors).
* Turn off visualization when you do not need it.
* For lidar, reduce the number of samples or resolution before increasing scene
  complexity.
* If a GPU-backed lidar fits your use case, consider `gpu_lidar` in
  [Sensors](sensors). Its availability and performance depend on rendering
  engine support and your GPU / driver setup.

### Reduce rendering work

* Jetty uses Ogre 2 by default. If you are forcing Ogre 1, compare it with the
  default first. If Ogre 2 is unavailable on your machine, [Troubleshooting](troubleshooting)
  explains the OpenGL requirements and how to fall back to `--render-engine ogre`.
* Reduce expensive visual effects when they are not needed. For example, disable
  shadow casting on lights with `<cast_shadows>false</cast_shadows>` and prefer
  simpler materials if rendering is the bottleneck.
* If your scene uses PBR materials and rendering is the bottleneck, test simpler
  materials or Ogre 1 as a comparison point. PBR is supported by render engines
  such as Ogre 2, but it is not free.

### Use simpler models

* Reduce triangle counts in imported meshes.
* Keep visual meshes and collision meshes separate so collisions can stay simple
  even when the visual model is detailed.
* Remove sensors, lights, or plugins from models that are not needed in a given
  experiment.

### Make sure Gazebo is using the right GPU

* On hybrid Intel / Nvidia systems, Gazebo may start on the integrated GPU.
  [Troubleshooting](troubleshooting) shows how to use PRIME render offload or
  Nvidia performance mode.
* If Ogre 2 fails to start, check OpenGL support and driver setup before
  assuming the simulation itself is slow.

## Advanced and external options

These are worth exploring for specialized workloads, but they are not part of a
small Jetty-only tuning guide:

* Alternative physics backends such as [Bullet Featherstone](https://github.com/gazebosim/gz-physics/issues/44)
  may help some articulated systems, but they need workload-specific validation
  and are not covered by a Jetty how-to yet.
* Third-party accelerators such as [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)
  or [gz_wgpu_rt_lidar](https://github.com/arjo129/gz_wgpu_rt_lidar/)
  may be useful when GPU-accelerated sensing is the main bottleneck. Follow the
  setup and support guidance in those projects.

Start with the documented knobs in Jetty first: physics step size, simpler
collisions, lower sensor rates, rendering settings, and correct GPU usage.
Those changes are usually easier to validate and easier to maintain across
releases.