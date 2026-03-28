# ROS2 + Gazebo Docker Bridge

A two-container Docker setup for running Gazebo and ROS2 as separate services connected over a custom Docker network, with the ros_gz_bridge handling topic communication between them

---

## Compatibility

| ROS2 | Gazebo |
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
  - `gazebo` — from `dockerfile.gazebo`
  - `ros2` — from your ROS2 Dockerfile
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
 
> Add these to `~/.bashrc` in each container to be available across sessions.

---

### 5. Install the Bridge

```bash
apt-get update && apt-get install -y ros-<ros-distro>-ros-gz-bridge
```

---

## Using a Config File (optional)

if you have a `bridge.yaml` config file, you can use it instead of passing topics as arguments:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/path/to/bridge.yaml
```

a typical `bridge.yaml` looks like this:

```yaml
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
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
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```
 
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

---

## Final Network Info
 
| Container | IP | Role |
|---|---|---|
| `gazebo-container` | `<gz-ip>` | Runs Gazebo simulation |
| `ros2-container` | `<ros-ip>` | Runs ROS2 + bridge |
 
Network name: `ros-link`