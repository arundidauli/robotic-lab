#!/usr/bin/env python3
"""Day 02: Forward Kinematics & Gripper Frames for SO-101.

Computes end-effector position and orientation from joint angles using
chained 4x4 homogeneous transformations and draws the live RGB frame axes.
"""

import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = os.path.join(os.path.dirname(__file__), "scene_task.xml")


def quat_to_mat(q):
    """Convert MuJoCo quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x**2 + y**2)],
    ], dtype=np.float64)


def rot_z_4x4(theta):
    """4x4 homogeneous transformation for pure rotation around local Z axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c,   -s,  0.0, 0.0],
        [s,    c,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)


def make_transform(R, p):
    """Construct a 4x4 homogeneous transformation from 3x3 rotation and 3D translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class SO101ForwardKinematics:
    """Analytical Forward Kinematics for SO-101 6-DOF arm."""

    def __init__(self, model):
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist")

        body_names = ["base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper"]
        bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in body_names]

        # Extract fixed relative link-to-link transforms from the model
        self.T_world_base    = make_transform(quat_to_mat(model.body_quat[bids[0]]), model.body_pos[bids[0]])
        self.T_base_shoulder = make_transform(quat_to_mat(model.body_quat[bids[1]]), model.body_pos[bids[1]])
        self.T_shoulder_upper= make_transform(quat_to_mat(model.body_quat[bids[2]]), model.body_pos[bids[2]])
        self.T_upper_lower   = make_transform(quat_to_mat(model.body_quat[bids[3]]), model.body_pos[bids[3]])
        self.T_lower_wrist   = make_transform(quat_to_mat(model.body_quat[bids[4]]), model.body_pos[bids[4]])
        self.T_wrist_gripper = make_transform(quat_to_mat(model.body_quat[bids[5]]), model.body_pos[bids[5]])

        # Tool center point (TCP) site offset relative to gripper body
        self.T_gripper_site  = make_transform(quat_to_mat(model.site_quat[self.site_id]), model.site_pos[self.site_id])

    def forward(self, q):
        """Compute end-effector 4x4 transform T_world_gripper from joint angles q."""
        q1, q2, q3, q4, q5 = q[:5]

        # --- The transform chain: one line per joint ---
        T_base     = self.T_world_base
        T_shoulder = T_base     @ self.T_base_shoulder  @ rot_z_4x4(q1)
        T_upper    = T_shoulder @ self.T_shoulder_upper @ rot_z_4x4(q2)
        T_lower    = T_upper    @ self.T_upper_lower    @ rot_z_4x4(q3)
        T_wrist    = T_lower    @ self.T_lower_wrist    @ rot_z_4x4(q4)
        T_gripper  = T_wrist    @ self.T_wrist_gripper  @ rot_z_4x4(q5)
        T_tool     = T_gripper  @ self.T_gripper_site   # tool frame offset

        return {
            "T_base": T_base,
            "T_wrist": T_wrist,
            "T_gripper": T_gripper,
            "T_tool": T_tool,
            "pos": T_tool[:3, 3],
            "rot": T_tool[:3, :3],
        }


def print_comparison_table(model, data, fk):
    """Evaluate and print FK vs MuJoCo site pose across 3 configurations."""
    test_configs = [
        ("Config 1 (Home / Zeros)", [0.0, 0.0, 0.0, 0.0, 0.0]),
        ("Config 2 (Reach / Pick)",  [0.45, -0.35, 0.55, -0.25, 0.60]),
        ("Config 3 (High / Angled)", [-0.75, 0.65, -0.45, 0.35, -1.10]),
    ]

    print("=" * 80)
    print("Day 02 — Forward Kinematics vs MuJoCo Site Pose Verification")
    print("=" * 80)

    for name, q in test_configs:
        data.qpos[:5] = q
        mujoco.mj_forward(model, data)

        res = fk.forward(q)
        fk_pos = res["pos"]
        fk_rot = res["rot"]

        mj_pos = data.site_xpos[fk.site_id]
        mj_rot = data.site_xmat[fk.site_id].reshape(3, 3)

        pos_diff = np.linalg.norm(fk_pos - mj_pos)
        rot_diff = np.linalg.norm(fk_rot - mj_rot)

        # Tool offset from wrist
        wrist_pos = res["T_wrist"][:3, 3]
        tool_offset = np.linalg.norm(fk_pos - wrist_pos)

        print(f"\n▶ {name}")
        print(f"  Joint angles (rad): {[round(x, 3) for x in q]}")
        print(f"  FK computed position:     [{fk_pos[0]:+.5f}, {fk_pos[1]:+.5f}, {fk_pos[2]:+.5f}] m")
        print(f"  MuJoCo site_xpos:         [{mj_pos[0]:+.5f}, {mj_pos[1]:+.5f}, {mj_pos[2]:+.5f}] m")
        print(f"  Position error norm:      {pos_diff:.2e} m  ({'EXACT' if pos_diff < 1e-12 else 'MISMATCH'})")
        print(f"  Rotation matrix diff:     {rot_diff:.2e}")
        print(f"  Wrist pos -> Tool pos:    offset = {tool_offset * 100:.2f} cm (tool != wrist!)")

    print("\n" + "=" * 80)


def draw_frame_axes(user_scn, origin, R, length=0.06, width=0.003):
    """Draw RGB coordinate arrows (X=Red, Y=Green, Z=Blue) at origin with orientation R."""
    colors = [
        [1.0, 0.1, 0.1, 0.9],  # X - Red
        [0.1, 0.9, 0.1, 0.9],  # Y - Green
        [0.1, 0.3, 1.0, 0.9],  # Z - Blue
    ]
    for i in range(3):
        target = origin + length * R[:, i]
        mujoco.mjv_connector(user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_ARROW, width, origin, target)
        user_scn.geoms[i].rgba = colors[i]
    user_scn.ngeom = 3


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"Error: model not found at {MODEL_PATH}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    fk = SO101ForwardKinematics(model)

    # 1. Print verification table
    print_comparison_table(model, data, fk)

    # Parse command line flags
    no_gui = "--no-gui" in sys.argv
    auto_exit = None
    if "--auto-exit" in sys.argv:
        auto_exit = float(sys.argv[sys.argv.index("--auto-exit") + 1])

    if no_gui:
        print("\nVerification complete (--no-gui specified). Exiting cleanly.")
        return

    # 2. Interactive / Visual playback in viewer
    print("\nLaunching viewer with live FK frame overlay...")
    print("Press ESC or close viewer window to exit.")

    waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.45, -0.35, 0.55, -0.25, 0.60]),
        np.array([-0.75, 0.65, -0.45, 0.35, -1.10]),
        np.array([0.20, -0.60, 0.80, -0.50, 0.00]),
        np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
    ]

    start_time = time.time()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        seg_idx = 0
        seg_steps = 120
        step_idx = 0

        while viewer.is_running():
            # Smooth cosine interpolation between waypoints
            q_start = waypoints[seg_idx]
            q_end = waypoints[(seg_idx + 1) % len(waypoints)]
            alpha = 0.5 * (1.0 - np.cos(np.pi * (step_idx / seg_steps)))
            q_current = (1.0 - alpha) * q_start + alpha * q_end

            # Apply to simulation
            data.qpos[:5] = q_current
            mujoco.mj_forward(model, data)

            # Compute analytical FK
            res = fk.forward(q_current)
            fk_pos = res["pos"]
            fk_rot = res["rot"]

            # Error check against MuJoCo site pose
            mj_pos = data.site_xpos[fk.site_id]
            err = np.linalg.norm(fk_pos - mj_pos)

            # Draw RGB frame axes live at computed hand pose
            with viewer.lock():
                draw_frame_axes(viewer.user_scn, fk_pos, fk_rot, length=0.06, width=0.003)

            # Update text overlay in viewer
            viewer.set_texts((
                mujoco.mjtFontScale.mjFONTSCALE_150,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                "Day 02  |  Forward Kinematics (Gripper Frame)",
                f"Hand Pos: [{fk_pos[0]:+.3f}, {fk_pos[1]:+.3f}, {fk_pos[2]:+.3f}] m  |  Err: {err*1e3:.4f} mm",
            ))

            viewer.sync()
            time.sleep(1 / 40)

            step_idx += 1
            if step_idx >= seg_steps:
                step_idx = 0
                seg_idx = (seg_idx + 1) % len(waypoints)

            if auto_exit is not None and (time.time() - start_time) > auto_exit:
                break


if __name__ == "__main__":
    main()
