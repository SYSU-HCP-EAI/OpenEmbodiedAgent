# Robot Embodiment Declaration

> Profile: franka_research3 | Driver: FrankaDriver / FrankaMultiBackendDriver

This file describes the physical capabilities and constraints of the Franka Research 3 (FR3) robot.
The Critic Agent reads this file to validate whether proposed actions are safe and feasible.

## Identity

- **Name**: Franka Research 3 (FR3)
- **Type**: 7-DOF Collaborative Robotic Arm with 2-Finger Gripper
- **Controller Interface**: Franka Control Interface (FCI) via pylibfranka
- **Supported Drivers**: `franka_research3` (pylibfranka), `franka_multi` (multi-backend negotiation)

## Network Architecture

```
WorkStation PC (pylibfranka) --> Control Box (Shop Floor Interface: 172.16.0.x)
                                          |
                                          +--> Robot Arm (internal: 192.168.0.x)
```

**Important**: You connect to the **Control Box's Shop Floor Interface** (e.g., `172.16.0.2`),
NOT directly to the Robot Arm. The Control Box then communicates with the arm via its internal network.

## Degrees of Freedom

| Joint | Range (rad) | Description |
|-------|------------|-------------|
| joint1 | ±2.8973 | Shoulder pan |
| joint2 | ±2.8973 | Shoulder lift |
| joint3 | ±2.8973 | Elbow |
| joint4 | ±2.8973 | Wrist 1 |
| joint5 | ±2.8973 | Wrist 2 |
| joint6 | ±2.8973 | Wrist 3 |
| joint7 | ±2.8973 | Fext (redundant) |

## Gripper Specifications

| Parameter | Value |
|-----------|-------|
| Model | Franka 2-Finger Gripper |
| Max width | 0.08 m (80 mm) |
| Max grasping force | 70 N |
| Default grasping force | 20 N |

## Supported Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `connect_robot` | `robot_id` (optional) | Establish connection to Franka Control Box |
| `check_connection` | `robot_id` (optional) | Run health check and update runtime state |
| `disconnect_robot` | `robot_id` (optional) | Close connection and stop motion |
| `stop` | `robot_id` (optional) | Immediately stop all motion |
| `move_to` | `x, y, z, roll, pitch, yaw, speed` (optional), `duration` (optional) | Move end-effector to Cartesian pose |
| `move_joints` | `q1-q7` or `joints` dict, `speed` (optional), `duration` (optional) | Move arm to joint configuration |
| `grasp` | `width, force` (optional), `speed` (optional) | Close gripper to grasp object |
| `move_gripper` | `width, speed` (optional) | Move gripper fingers to target width |
| `get_robot_state` | `robot_id` (optional) | Return current robot state snapshot |

## Physical Constraints

| Parameter | Value |
|-----------|-------|
| Max reach | 855 mm from base |
| Max payload | 3 kg |
| Repeatability | ±0.1 mm |
| Max Cartesian speed | 0.5 m/s (configurable) |
| Max joint velocity | 2.175 rad/s per joint |
| Collision behavior | Default: stop and release impedance |
| Home position/Safe position (Cartesian space)| position: [0.307, 0.0, 0.487], orientation: [1.0, 0.0, 0.0, 0.0] (configurable)|
| Home position/Safe position (joint space)| [0.0, -math.pi / 4, 0.0, -3*math.pi / 4, 0.0, math.pi / 2, math.pi / 4] (configurable)|


## Connection

- **Transport**: Ethernet (TCP/IP via pylibfranka)
- **Default IP**: `172.16.0.2` (Shop Floor Interface)
- **Control rate**: 250 Hz (default), configurable up to 1000 Hz
- **Realtime mode**: `false` by default (use `kIgnore` for development; set `true` for 1kHz real-time control)
- **Auto-discover**: Tries known Control Box IPs if no IP is provided
- **Environment variables**: `PAOS_FRANKA_IP`, `PAOS_FRANKA_CONTROL_RATE`, `PAOS_FRANKA_ROBOT_ID`, `PAOS_FRANKA_GRIPPER_FORCE`
- **Reconnect Policy**: `auto` (default)

## Safety Notes

- **Collision detection**: Enabled by default; stops motion on contact
- **FCI activation**: Must be activated via Desk interface on the Control Box
- **Joint limits**: Hard limits at ±2.8973 rad
- **Network**: Control PC must be on same subnet as Control Box (e.g., 172.16.0.x)
- **Physical limits**: see [official document](https://frankarobotics.github.io/docs/robot_specifications.html)

## Prerequisites

Before first use:

1. **Activate FCI**: On the Control Box Desk interface, go to sidebar → **Activate FCI**
2. **Network**: Connect Control PC to Control Box via Ethernet, set IP to `172.16.0.x` (e.g., `172.16.0.1`)
3. **Install backend drivers**:

```bash
# Recommended: Install both backends for automatic negotiation
pip install pylibfranka                                          # Official Python bindings
pip install git+https://github.com/TimSchneider42/franky.git    # Alternative high-level library
```

For **real-time control** (1 kHz, strict timing):
- Install real-time kernel on Control PC
- Set `realtime_mode: true` in config

### Backend Selection

When using `franka_multi` driver, the system automatically negotiates between backends:

| Backend | Version Compatibility | Capabilities |
|---------|----------------------|--------------|
| `franky` | More relaxed, recommended for Robot System 9 | Basic motion control (no force/impedance) |
| `pylibfranka` | Exact version match required | Full capabilities including force/impedance |

Use `force_backend` option to explicitly select a backend if needed.

## Runtime Protocol

- **Connection channel**: `robots.<robot_id>.connection_state`
- **Pose channel**: `robots.<robot_id>.robot_pose`
- **Joint channel**: `robots.<robot_id>.joint_state`
- **Arm control channel**: `robots.<robot_id>.arm_state`
- **Gripper channel**: `robots.<robot_id>.gripper_state`
- **Health owner**: `hal_watchdog.py` via periodic `health_check()`

## Notes

- Higher-level behaviors (task planning, visual servoing) should be composed by OEA workflow layers.
- The driver supports joint position, Cartesian pose, and gripper control modes.
- Force/torque control and advanced impedance modes require pylibfranka backend (not available in franky).
- See [official document](https://frankarobotics.github.io/docs/compatibility.html) and [franka_compatibility.md](../../docs/user_manual/appendix/franka_compatibility.md) for version compatibility details.
- See [franka_version_guide.md](../../docs/user_manual/appendix/franka_version_guide.md) for installation and upgrade guides.