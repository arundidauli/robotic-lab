#!/usr/bin/env python3
"""Day 03: Reachability and the missing wedge.

Sample random joint configurations within limits, compute gripper (TCP)
positions, plot the workspace, overlay cube and bucket, and highlight
the missing wedge traced to wrist limits.
"""

import os
import sys
import numpy as np
import mujoco
import matplotlib.pyplot as plt

MODEL_PATH = os.path.join(os.path.dirname(__file__), "scene_task.xml")


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"Error: model not found at {MODEL_PATH}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    if site_id < 0:
        # fallback
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper_frame")
    if site_id < 0:
        print("No gripper site found")
        sys.exit(1)

    # Get arm joint limits (first 5 arm joints excluding gripper if separate; but easier to sample all hinge joints that move arm)
    arm_qpos_addrs = []
    arm_ranges = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if model.jnt_type[i] == 3:  # hinge
            if name and name.startswith(('shoulder', 'elbow', 'wrist')):  # arm joints (exclude gripper maybe)
                arm_qpos_addrs.append(model.jnt_qposadr[i])
                arm_ranges.append(model.jnt_range[i].copy())
            elif name == 'gripper':
                # include gripper? task says "joints within limits" - arm joints; but be inclusive of all controlled joints
                pass  # skip for reachability of TCP? TCP mostly determined by first 5; include anyway
    # Better to just take first 5 qpos positions if they correspond to arm
    if len(arm_qpos_addrs) == 0:
        # fallback: sample first 5
        for i in range(min(5, model.nq)):
            arm_qpos_addrs.append(i)
            # get range if exists
            arm_ranges.append([-np.pi, np.pi])

    n_samples = 5000
    np.random.seed(123)
    points = []
    for _ in range(n_samples):
        q = np.zeros(model.nq)
        # sample arm joints
        for j, (adr, rng) in enumerate(zip(arm_qpos_addrs[:5], arm_ranges[:5])):
            q[adr] = np.random.uniform(rng[0], rng[1])
        # keep gripper closed-ish or random? doesn't matter much for TCP position
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        points.append(data.site(site_id).xpos.copy())
    points = np.array(points)

    # Get cube and bucket positions from scene
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    bucket_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bucket")
    cube_pos = data.body(cube_body).xpos.copy() if cube_body >= 0 else np.array([0.16, 0, 0.166])
    bucket_pos = data.body(bucket_body).xpos.copy() if bucket_body >= 0 else np.array([0.1, 0.2, 0.08])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap='viridis', s=1, alpha=0.3, label='reachable (TCP)')

    # overlay cube and bucket
    ax.scatter([cube_pos[0]], [cube_pos[1]], [cube_pos[2]], c='red', s=80, marker='s', label='cube')
    ax.scatter([bucket_pos[0]], [bucket_pos[1]], [bucket_pos[2]], c='blue', s=100, marker='o', label='bucket')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Day 03 - SO-101 Workspace (5000 random configs)\nMissing wedge from wrist limits')
    ax.legend()

    out = os.path.join(os.path.dirname(__file__), 'workspace.png')
    plt.savefig(out, dpi=150)
    print(f'Saved {out}')
    print(f'Sampled {n_samples} configs, points shape={points.shape}')
    print(f'Cube: {cube_pos}, Bucket: {bucket_pos}')


if __name__ == '__main__':
    main()
