# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Installation and Setup

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
pip install -e .
conda install -c conda-forge libstdcxx-ng -y
```

SMPL-X body models must be placed at `assets/body_models/smplx/` for SMPL-X/AMASS input support.

Run all Python commands inside the conda env: `conda run -n gmr python ...` or activate first.

## Common Commands

```bash
# SMPL-X (AMASS/OMOMO) to robot
python scripts/smplx_to_robot.py --smplx_file <path.npz> --robot unitree_g1 --save_path <out.pkl>

# BVH (LAFAN1/Rokoko) to robot
python scripts/bvh_to_robot.py --bvh_file <path.bvh> --robot unitree_g1 --format lafan1 --save_path <out.pkl>

# Xsens BVH offline
python scripts/xsens_bvh_to_robot.py --bvh_file <path.bvh> --robot unitree_g1

# GVHMR (monocular video pose estimation) to robot
python scripts/gvhmr_to_robot.py --gvhmr_pred_file <hmr4d_results.pt> --robot unitree_g1

# OptiTrack FBX offline
python scripts/fbx_offline_to_robot.py --motion_file <path.pkl> --robot unitree_g1

# Visualize saved motion
python scripts/vis_robot_motion.py --robot unitree_g1 --robot_motion_path <out.pkl>

# Real-time: OptiTrack live
python scripts/optitrack_to_robot.py --server_ip <ip> --client_ip <ip>

# Real-time: Xsens MVN live
python scripts/xsens_live_streaming.py --robot unitree_g1
```

Add `--record_video --video_path output.mp4` to visualization scripts to record.  
Add `--loop` to loop a motion file indefinitely.

## Architecture

### Data Flow

```
Human motion source
    │
    ▼
Format-specific loader (utils/smpl.py, utils/lafan1.py, utils/xsens.py)
    │  produces: list of frame dicts {body_name: (position_xyz, quat_wxyz)}
    ▼
GeneralMotionRetargeting.retarget(frame_dict)
    │  applies: scale → rotation offsets → IK
    ▼
qpos array: [pos(3), quat_wxyz(4), joint_angles(N)]
    │
    ▼
Saved to .pkl: {fps, root_pos(Nx3), root_rot(Nx4 xyzw), dof_pos(NxJ)}
```

### Core Classes

- **`GeneralMotionRetargeting`** (`motion_retarget.py`): Wraps mink IK solver. Initialized with `src_human` (format key) and `tgt_robot` (robot key). The `retarget(frame)` method runs two sequential IK passes and returns `qpos`.
- **`RobotMotionViewer`** (`robot_motion_viewer.py`): MuJoCo passive viewer. Call `step(root_pos, root_rot, dof_pos, ...)` each frame.
- **`KinematicsModel`** (`kinematics_model.py`): Torch-based FK for converting joint angles to body positions (used for downstream RL training pipelines, not for the retargeting itself).
- **`params.py`**: Single source of truth for robot→XML path (`ROBOT_XML_DICT`), format+robot→IK config path (`IK_CONFIG_DICT`), and robot→base body name (`ROBOT_BASE_DICT`).

### Two-Pass IK

`ik_match_table1` runs first (orientation-heavy, no position for pelvis), then `ik_match_table2` (position-dominant). This improves convergence: limbs are oriented correctly before the pelvis position is locked down.

### Input Format Keys

Used as `src_human` in `GeneralMotionRetargeting` and as keys in `IK_CONFIG_DICT`:

| Key | Source | Loader |
|-----|--------|--------|
| `smplx` | AMASS/OMOMO `.npz` | `utils/smpl.py` |
| `bvh_lafan1` | LAFAN1 dataset or Rokoko LAFAN1 export | `utils/lafan1.py` |
| `bvh_nokov` | Nokov mocap BVH | `utils/lafan1.py` (format="nokov") |
| `bvh_xsens` | Xsens offline BVH | `utils/xsens.py` |
| `fbx` | OptiTrack live/FBX | `optitrack_vendor/` |
| `fbx_offline` | OptiTrack offline pre-processed pkl | direct pkl load |
| `xrobot` | XRoboToolkit SDK live stream | `xrobot_utils.py` |
| `xsens_mvn` | Xsens MVN live stream | `utils/xsens_vendor/` |

### IK Config Files (`ik_configs/`)

Named `{format}_to_{robot}.json`. Schema for each entry in `ik_match_table1` / `ik_match_table2`:

```json
"robot_body_name": [
    "human_body_name",
    <position_weight>,
    <rotation_weight>,
    [pos_x_offset, pos_y_offset, pos_z_offset],
    [w, x, y, z]   // rotation offset quaternion (scalar-first) applied to human body rotation
]
```

**Critical**: Rotation offsets convert from the human skeleton's local frame (after any coordinate conversion) to the robot joint frame. For BVH formats, `lafan1.py` first applies an Rx(+90°) coordinate transform (Y-up→Z-up), so offsets must account for this. See `DOC.md` for full annotation.

### Adding a New Robot

1. Add robot XML path to `ROBOT_XML_DICT` in `params.py`
2. Add base body name to `ROBOT_BASE_DICT` and camera distance to `VIEWER_CAM_DISTANCE_DICT`
3. Create `ik_configs/{format}_to_{robot}.json` by matching human body names to robot body names and computing rotation offsets from T-pose alignment
4. Add entries to `IK_CONFIG_DICT` in `params.py`

### Coordinate Systems

- **BVH files**: Y-up. `lafan1.py` applies `rotation_matrix = [[1,0,0],[0,0,-1],[0,1,0]]` (Rx+90°) to convert positions and orientations to MuJoCo Z-up space.
- **SMPL-X**: No coordinate conversion applied; rotation offsets in the IK config handle frame alignment directly.
- **MuJoCo / robot**: Z-up, +X forward (for most robots).
- **Quaternion convention**: All quaternions in the codebase use **scalar-first** (w, x, y, z) except the pkl output `root_rot` which is saved as xyzw for compatibility.

### Output pkl Format

```python
{
    "fps": int,
    "root_pos": np.ndarray,   # (N, 3) — pelvis world position
    "root_rot": np.ndarray,   # (N, 4) — pelvis quaternion, xyzw order
    "dof_pos": np.ndarray,    # (N, num_joints)
    "local_body_pos": None,   # reserved
    "link_body_list": None,   # reserved
}
```

Load with `from general_motion_retargeting import load_robot_motion`.
