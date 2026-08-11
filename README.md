# TurtleBot4 Waypoint Navigation & Sphere Detection — Recruitment Task

***Simulation video link***


In this video the largest object is RED
Navigate to websocket_broadcaster.py and change the Waypoint co-ordinates to get a change in result


A ROS 2 project built on top of a TurtleBot4 simulation (`SadmanSyfe/Recruitment-task`
template): the robot receives a sequence of waypoints over a WebSocket
connection, drives to each one in order, and at the final waypoint uses its
camera to detect and report the color of the largest colored sphere in view.
Optional bonus: live SLAM mapping.

---

## 1. Task Summary

| Phase | What it does |
|---|---|
| **2.1 — WebSocket client** | Connect to `ws://localhost:8765`, receive a JSON list of waypoints (each with `x`, `y`, `yaw`) broadcast by the provided `websocket_broadcaster` node |
| **2.2 — Go-to-goal navigation** | Drive the robot through all waypoints in order using distance/heading error control on `/cmd_vel` (no Nav2) — rotate-to-heading → drive → rotate-to-final-yaw per waypoint |
| **2.3 — Sphere color detection** | At the final waypoint, grab a camera frame, use HSV thresholding + contour detection to find colored spheres, identify the one with the largest contour area, and log its color |
| **Bonus — SLAM** | Run `slam_toolbox` alongside navigation for live occupancy-grid mapping |
| **Submission** | Dockerized solution + README with exact run commands + narrated video walkthrough |

---

## 2. Environment

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 (WSL2 on Windows) |
| ROS 2 | Humble |
| Simulator | Ignition Gazebo (Fortress-era, via `turtlebot4_ignition_bringup` — **not** `turtlebot4_gz_bringup`, despite that being the name used in some TurtleBot4 docs/templates) |
| Robot model | TurtleBot4 Lite |
| World | `level3.sdf` (provided by the `questions` package) |
| Camera topic | `/oakd/rgb/preview/image_raw` |

> **Naming gotcha:** the assignment's launch file was originally written
> against `turtlebot4_gz_bringup` (Harmonic-naming), but this environment's
> installed TurtleBot4 release uses the older **Ignition** naming —
> `turtlebot4_ignition_bringup`. If you hit
> `PackageNotFoundError: turtlebot4_gz_bringup`, check
> `ros2 pkg list | grep turtlebot4` for the actual installed package name
> and update the launch file's `get_package_share_directory(...)` call
> accordingly.

---

## 3. Repo Structure

```
Recruitment-task/
├── level3.sdf                          # copied here at launch time (see note below)
├── src/
│   └── questions/
│       ├── launch/
│       │   └── main_assignment.launch.py   # spawns robot + starts websocket_broadcaster
│       ├── worlds/
│       │   └── level3.sdf
│       └── questions/
│           ├── websocket_broadcaster.py    # provided — serves waypoints on ws://0.0.0.0:8765
│           └── waypoint_navigator.py       # your code — client + go-to-goal + CV
├── slam_config/
│   └── mapper_params_online_async.yaml     # SLAM bonus config
├── Dockerfile
├── docker-compose.yml
└── docker/
    └── entrypoint.sh
```

**Important quirk:** `main_assignment.launch.py` copies `level3.sdf` from
the installed package share directory into your **current working
directory** at launch time (a workaround for a Gazebo resource-lookup
issue). This means:
- Always run `ros2 launch` from a directory you have write access to.
- If you edit the world (e.g. sphere positions/colors for testing), edit
  the **installed** copy under the `questions` package's share folder, not
  the copy that lands in your CWD — the CWD copy gets silently
  overwritten on every launch.

---

## 4. Native Setup (no Docker)

```bash
# Clone
git clone https://github.com/SadmanSyfe/Recruitment-task.git ~/Recruitment-task
cd ~/Recruitment-task

# Install TurtleBot4 simulator packages
sudo apt update
sudo apt install ros-humble-turtlebot4-desktop ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation

# If you hit a Gazebo bridge package conflict (ros-gzharmonic-* vs ros-gz-*):
sudo apt full-upgrade -y
# if that alone doesn't resolve it:
sudo apt remove 'ros-humble-ros-gzharmonic*'
sudo apt install ros-humble-turtlebot4-desktop ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

> Every new terminal needs `source /opt/ros/humble/setup.bash` **and**
> `source install/setup.bash` (run from `~/Recruitment-task`) before any
> `ros2` command will see the `questions` package.

### Run (native, 2 terminals)

**Terminal 1 — simulation + waypoint broadcaster:**
```bash
cd ~/Recruitment-task
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch questions main_assignment.launch.py
```
Wait for the Gazebo window to fully load before starting Terminal 2.

**Terminal 2 — navigator:**
```bash
cd ~/Recruitment-task
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run questions waypoint_navigator
```

Expected log sequence: `Connected to ws://localhost:8765` →
`Received 13 waypoints from server.` → a `Reached waypoint N/13 (...)` line
per waypoint → sphere contour areas logged at the end, e.g.
`Sphere contour areas (px^2): {'RED': ..., ...}`.

### Sanity checks

```bash
# Confirm the broadcaster is actually listening
ss -tlnp | grep 8765

# Confirm the camera topic exists and is publishing
ros2 topic list | grep -i image
ros2 topic hz /oakd/rgb/preview/image_raw
```

---

## 5. Docker Setup

The solution is packaged as **one image, two services** — `sim` and
`navigator` — both built from the same Dockerfile, differentiated at
runtime by the command passed to `docker/entrypoint.sh` (a `case "$1" in
sim|navigator) ... esac` dispatch), not by separate `CMD`s.

### Build

```bash
cd ~/Recruitment-task
sudo docker compose build sim navigator
```

### Run (2 terminals)

```bash
# Terminal 1
sudo docker compose up sim

# Terminal 2 (after sim's Gazebo window fully loads)
sudo docker compose up navigator
```

### Cross-container ROS 2 discovery (important)

By default, ROS 2's discovery relies on multicast, which is unreliable
between separate Docker containers on WSL2's virtualized network — even
with `network_mode: host` set on both services. Symptoms: `sim` and
`navigator` each start cleanly, but `ros2 node list` inside one container
never shows the other's nodes, and `/cmd_vel` never reaches the robot.

**Fix applied:** switch the RMW implementation to CycloneDDS and force
unicast discovery to `127.0.0.1` (valid since both containers share the
host network namespace). In the Dockerfile:
```dockerfile
RUN apt-get update && apt-get install -y \
    ...
    ros-humble-rmw-cyclonedds-cpp \
    ...
```
And in `docker-compose.yml`, under a shared `environment:` block for both
services:
```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONEDDS_URI=<path to a CycloneDDS XML config pointing peers at 127.0.0.1>
  - ROS_DOMAIN_ID=<same value for both services>
```

To verify discovery is actually working after this fix:
```bash
sudo docker exec -it tb4_navigator bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
# should show nodes from BOTH containers, e.g. /waypoint_navigator AND /turtlebot4_node

sudo docker exec -it tb4_sim bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /cmd_vel"
# should show live Twist messages once the navigator is driving
```

### Other Docker-specific fixes baked in

- **`numpy<2` pinned in the navigator's pip install** — a newer numpy
  breaks `cv_bridge` (`AttributeError: _ARRAY_API not found`) inside the
  container image used here.
- **GL/rendering libs** (`mesa-utils`, `libgl1-mesa-dri`, `libgl1-mesa-glx`)
  installed for Gazebo's GUI to render inside the container.

---

## 6. Known Issues / Current Status

| Issue | Status |
|---|---|
| `turtlebot4_gz_bringup` not found | **Fixed** — environment uses `turtlebot4_ignition_bringup` instead; launch file updated to match |
| `ros-gzharmonic-*` vs `ros-gz-*` apt conflict | **Fixed** — resolved via `apt full-upgrade` / manual removal of the stale harmonic-named packages |
| Camera topic mismatch in navigator code | **Fixed** — corrected to `/oakd/rgb/preview/image_raw` |
| Waypoint 13's given coordinates appeared to lead off the intended path | **Patched** — coordinate correction applied in `waypoint_navigator.py`; flagged as a possible discrepancy in the assignment's own waypoint data, not a bug in the navigator logic |
| `websocket_broadcaster` process alive but nothing listening on 8765 | **Fixed** — was a silent failure in the server's background thread; resolved after rebuild |
| Native (non-Docker) run: robot moves correctly through all 13 waypoints, sphere detection logs contour areas | **Working** |
| Docker: cross-container ROS 2 discovery (`sim` ↔ `navigator`) failing over default multicast | **Fixed** — switched to CycloneDDS RMW with unicast peer config (Section 5) |
| Docker: `cv_bridge` crash from numpy 2.x incompatibility | **Fixed** — `numpy<2` pinned in Dockerfile |
| Docker: robot not moving even after discovery fix, mid-verification | **Unresolved / in progress** — last confirmed step was rebuilding with CycloneDDS installed; `/cmd_vel` echo test across containers not yet confirmed working end-to-end |
| SLAM bonus (`slam_toolbox online_async_launch.py`) | **Partially working** — node runs and `/map` topic exists, but map wasn't visibly updating in RViz at last check; `base_frame`/`odom_frame`/`scan_topic` params were being verified when work paused |
| Docker daemon not persistent across WSL sessions | **Known limitation** — `dockerd` isn't running as a managed service in this environment (no systemd at the time this was hit); requires `sudo dockerd &` manually each session, or enabling systemd + `systemctl enable --now docker` |

---

## 7. Troubleshooting Notes

- **`ros2 launch questions ...` says package not found** — the workspace
  isn't built/sourced. `colcon build --symlink-install` then
  `source install/setup.bash`, every new terminal.
- **`Connect call failed ('127.0.0.1', 8765); retrying...`** — nothing is
  listening on the broadcaster's port. Confirm Terminal 1 (the sim +
  launch file, which starts `websocket_broadcaster`) is actually running;
  check with `ss -tlnp | grep 8765`.
- **`Duplicate package names not supported` on `colcon build`** — usually
  means the repo got cloned or copied into a nested folder (e.g.
  `Recruitment-task/Recruitment-task/src/questions`), so colcon finds two
  `questions` packages. Locate with
  `find ~/Recruitment-task -maxdepth 4 -type d -name questions` and remove
  the duplicate.
- **Docker: `permission denied .../docker.sock`** — user not in the
  `docker` group, or a stale session; `sudo usermod -aG docker $USER` then
  fully close/reopen the terminal.
- **Docker: `docker: unrecognized service` / `service docker start`
  fails** — WSL2 without systemd has no `service` command wired up for
  Docker. Either enable systemd in `/etc/wsl.conf`
  (`[boot]` / `systemd=true`, then `wsl --shutdown` from PowerShell), or
  run `sudo dockerd &` manually each session as a workaround.
- **`docker-compose.yml: the attribute 'version' is obsolete`** — harmless
  warning on newer Compose versions; safe to ignore, or remove the
  top-level `version:` key from the file.
