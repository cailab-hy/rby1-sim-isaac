# rby1-sim-isaac

Rainbow Robotics **RBY1** robot simulator powered by **NVIDIA Isaac Sim 5.1.0**.

<img width="2267" height="1275" alt="RBY1 robot in Isaac Sim" src="docs/Isaac_sim_scene.png?v=3" />

## Requirements

* NVIDIA GPU + driver
* [Docker](https://docs.docker.com/engine/install/)
* [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Minimal install check

```bash
nvidia-smi
docker --version
dpkg -l | grep -E 'nvidia-container-toolkit|nvidia-container-runtime'
```

If `nvidia-smi` prints the GPU table and the last command lists an NVIDIA
container package, the NVIDIA driver, Docker, and NVIDIA Container Toolkit are
installed.

## Quick start

### 1. Clone the repository

```bash
git clone https://github.com/RainbowRobotics/rby1-sim-isaac.git
cd rby1-sim-isaac
```

### 2. Run

| Script | Robot control | Robot UDP bridge | `app_main isaac` |
|--------|---------------|------------------|------------------|
| `docker/run.sh` | Built-in standalone PD trajectory | Disabled by default | Not started |
| `docker/run_sdk.sh` | rby1-sdk / rby1 core commands | Enabled via `--udp` | Started automatically |

#### rby1-sdk UDP integration mode

Connects rby1-sdk with the Isaac Sim container to enable robot control via rby1 core.

```bash
./docker/run_sdk.sh --image 0.10.7-a_v1.2   # Model A v1.2
./docker/run_sdk.sh --image 0.10.7-m_v1.2   # Model M v1.2
./docker/run_sdk.sh --image 0.10.7-a_v1.2 --gripper --gripper-name rb_gripper
./docker/run_sdk.sh --image 0.10.7-m_v1.2 --gripper --gripper-name rb_gripper
```

#### Standalone mode

Runs Isaac Sim only. The robot is controlled by the built-in PD trajectory in `RBY1Task`; no `app_main` process is started and the robot command/state UDP bridge is disabled.

```bash
./docker/run.sh --image 0.10.7-a_v1.2   # Model A v1.2
./docker/run.sh --image 0.10.7-m_v1.2   # Model M v1.2
GRIPPER_NAME=rb_gripper ./docker/run.sh --image 0.10.7-m_v1.2
```

The default launch uses the no-gripper base USD. Additional arguments after
`--image` are forwarded to `simulation.py`, including `--gripper`,
`--gripper-name <name>`, and `--no-sim-gripper`. A gripper can also be selected
and enabled with the `GRIPPER_NAME` environment variable.

### Custom gripper assets

The simulator loads gripper assets by folder name, so a different hand can be
attached without modifying the base RBY1 USD. Place the hand USD files and mount
configuration under `assets/gripper/<gripper-name>/`:

<img width="1280" height="900" alt="RBY1 custom gripper attachment structure" src="docs/gripper_attachment_overview.svg?v=2" />

```text
assets/gripper/<gripper-name>/
├── <gripper-name>_left.usd
├── <gripper-name>_right.usd
└── gripper.json
```

`gripper.json` defines which prim in each hand is fixed to the robot wrist:

```json
{
  "left": {
    "body": "link_mount_left",
    "mount_pos": [0.0, 0.0, -0.1261],
    "mount_rot": [0.0, 0.70710678, -0.70710678, 0.0]
  },
  "right": {
    "body": "link_mount_right",
    "mount_pos": [0.0, 0.0, -0.1261],
    "mount_rot": [0.0, 0.70710678, 0.70710678, 0.0]
  }
}
```

For example, a user-provided hand can be tested by placing USD files under
`assets/gripper/custom_gripper/` and launching:

```bash
./docker/run.sh --image 0.10.7-a_v1.2 --gripper --gripper-name custom_gripper
```

User-provided USD files are not included in this repository; only the attachment
pattern and `gripper_servers/custom_gripper.py` adapter skeleton are provided.

## Supported images

The following images are currently supported. Each image automatically loads the
USD asset matching its model (via the `ROBOT_MODEL_NAME` env var), so no extra
flag is required.

| Tag | Model |
|-----|-------|
| `0.10.7-a_v1.2` | Model A v1.2 |
| `0.10.7-m_v1.2` | Model M v1.2 |

## Docker image tag convention

```
rainbowroboticsofficial/rby1-sim-isaac:<version>-<model>_v<model_version>

e.g.) rainbowroboticsofficial/rby1-sim-isaac:0.10.7-a_v1.2   # Model A v1.2
      rainbowroboticsofficial/rby1-sim-isaac:0.10.7-m_v1.3   # Model M v1.3
```

## UDP ports

Since `--network=host` is used, the host and container share the same network namespace.
Ports `5005/5006` are used by rby1-sdk integration mode; ports `5007/5008` are used by the sim gripper bridge.

| Port | Direction | Description |
|------|-----------|-------------|
| `5005/udp` | Isaac Sim → rby1-sdk | RobotState (`ControlState`) |
| `5006/udp` | rby1-sdk → Isaac Sim | RobotCommand (`ControlInput`) |
| `5007/udp` | User code → Isaac Sim | Gripper command |
| `5008/udp` | Isaac Sim → User code | Gripper state |

## Gripper example

When Isaac Sim is running (with `--sim-gripper` enabled by default), you can directly control the gripper from the host.

```bash
cd examples
pip install numpy          # dependency

# In IsaacSim
python3 gripper_example.py --sim

# Real robot
python3 gripper_example.py
```

The `SimDynamixelBus` class in `sim_gripper_bridge.py` provides the same interface as `rby1_sdk.DynamixelBus`. By changing just one line (`--sim`), you can control both the real robot and simulation with the same code.

## License

Source files containing the NVIDIA SPDX header are licensed under Apache-2.0.
