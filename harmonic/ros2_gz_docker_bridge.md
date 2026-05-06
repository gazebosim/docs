# ROS 2 + Gazebo Docker Bridge

A two-container Docker setup for running Gazebo and ROS 2 as separate services connected over a custom Docker network, with the ros_gz_bridge handling topic communication between them

---

## Compatibility

| ROS 2 | Gazebo |
|---|---|
| Rolling | Jetty |
| Kilted | Ionic |
| Jazzy | Harmonic |
| Iron | Harmonic |
| Humble | Fortress |

---

## Prerequisites
 
- Docker installed and running
- Both images built:
  - `gazebo` — from your Gazebo Dockerfile
  - `ros2` — from your ROS 2 Dockerfile
- `ros-gz-bridge` installed

---

## Setup
 
### 1. Create the Docker Network
 
```bash
docker network create ros-link
```
 
### 2. Start Containers
 
```bash
docker run -it -d --name gazebo-container --network ros-link gazebo sleep infinity
docker run -it -d --name ros2-container --network ros-link ros2 sleep infinity
```

### 3. Check Container IPs
 

```bash
docker network inspect ros-link
```
 
Example output:
```json
[
  {
    "Name": "ros-link",
    "Driver": "bridge",
    "IPAM": {
      "Config": [{ "Subnet": "172.21.0.0/16", "Gateway": "172.21.0.1" }]
    },
    "Containers": {
      "281b3cbfba6a...": {
        "Name": "ros2-container",
        "IPv4Address": "172.21.0.3/16" //<ros-ip>
      },
      "c2410d610249...": {
        "Name": "gazebo-container",
        "IPv4Address": "172.21.0.2/16" //<gz-ip>
      }
    }
  }
]
```

---

### 4. Configure Environment Variables
 
**In `gazebo-container`:**
```bash
export GZ_IP=<gz-ip>
export GZ_PARTITION=gazebo-container
```
 
**In `ros2-container`:**
```bash
export GZ_IP=<ros-ip>
export GZ_PARTITION=gazebo-container
source /opt/ros/<ros-distro>/setup.bash
```

:::{important}
Add these to `~/.bashrc` in each container to be available across sessions.
:::

:::{note}
When setting `GZ_IP`, use only the IP address without the subnet mask. For example:

```bash
export GZ_IP=172.21.0.3
```

Do not include `/16`.
:::

---

### 5. Install the Bridge

```bash
apt-get update && apt-get install -y ros-<ros-distro>-ros-gz-bridge
```

---

## Running the Bridge
 
### Terminal 1 — Start Gazebo sim
 
```bash
docker exec -it gazebo-container bash
gz sim shapes.sdf
```
 
### Terminal 2 — Start the bridge
 
```bash
docker exec -it ros2-container bash
source /opt/ros/<ros-distro>/setup.bash
```

#### Option 1: Single topic (basic)
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

#### Option 2: Multiple topics
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /world/default/model/vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /world/default/model/vehicle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

#### Option 3: Using config file
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/path/to/bridge.yaml
```
See the [ros_gz_bridge examples](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge/examples) for sample configuration files.
 
### Terminal 3 — Verify
 
```bash
docker exec -it ros2-container bash
source /opt/ros/<ros-distro>/setup.bash
ros2 topic echo /clock
```
if you see something like this :- 
```bash
clock:
  sec: 0
  nanosec: 0
---
clock:
  sec: 0
  nanosec: 0
---
clock:
  sec: 0
  nanosec: 0
---
```
then the connection between the two docker containers is succesfull