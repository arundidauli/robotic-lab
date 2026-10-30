#!/usr/bin/env python3
"""Day 04: Position-only IK with damped least squares.

Solve IK to reach 10 sampled targets, log final error and iterations.
Show a target near a joint limit where the solver oscillates.
"""

import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

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
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper_frame")

    # Get arm joint info (first 5)
    arm_qpos = []
    arm_ranges = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if model.jnt_type[i] == 3 and name:
            if name.startswith(('shoulder', 'elbow', 'wrist')) or name in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']:
                arm_qpos.append(model.jnt_qposadr[i])
                arm_ranges.append(model.jnt_range[i].copy())
                if len(arm_qpos) >= 5:
                    break

    n_dof = min(5, len(arm_qpos))
    arm_qpos = arm_qpos[:n_dof]
    arm_ranges = arm_ranges[:n_dof]

    def get_tcp():
        mujoco.mj_forward(model, data)
        return data.site(site_id).xpos.copy()

    def jacobian_pos():
        # compute Jacobian for site in world frame, position part
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        # select rows for our joints
        J = np.zeros((3, n_dof))
        for i in range(n_dof):
            J[:, i] = jacp[:, arm_qpos[i]] if arm_qpos[i] < model.nv else 0
        return J

    def ik_solve(target, q0=None, max_iter=200, alpha=0.5, lam=1e-3, tol=1e-4):
        if q0 is None:
            q = np.zeros(n_dof)
            for i in range(n_dof):
                q[i] = (arm_ranges[i][0] + arm_ranges[i][1]) / 2.0
        else:
            q = q0.copy()
        for it in range(max_iter):
            for i in range(n_dof):
                data.qpos[arm_qpos[i]] = np.clip(q[i], arm_ranges[i][0], arm_ranges[i][1])
            mujoco.mj_forward(model, data)
            pos = get_tcp()
            err = target - pos
            err_norm = np.linalg.norm(err)
            if err_norm < tol:
                return q, err_norm, it + 1, True
            J = jacobian_pos()
            # Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 err
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + lam * lam * np.eye(3), err)
            q = q + alpha * dq
            # clamp
            for i in range(n_dof):
                q[i] = np.clip(q[i], arm_ranges[i][0], arm_ranges[i][1])
        return q, err_norm, max_iter, False

    # Sample 10 targets within workspace region
    np.random.seed(42)
    targets = []
    for _ in range(10):
        # sample near reachable region based on day01/02
        x = np.random.uniform(0.15, 0.35)
        y = np.random.uniform(-0.15, 0.15)
        z = np.random.uniform(0.05, 0.25)
        targets.append(np.array([x, y, z]))

    # Add one target near joint limit that causes oscillation
    targets.append(np.array([0.12, 0.18, 0.12]))  # likely near limit/wedge

    print("=" * 60)
    print("Day 04 - Position-only IK (damped least squares)")
    print("=" * 60)
    results = []
    for i, t in enumerate(targets):
        q, err, iters, conv = ik_solve(t)
        # apply and measure
        for j in range(n_dof):
            data.qpos[arm_qpos[j]] = q[j]
        mujoco.mj_forward(model, data)
        pos = get_tcp()
        results.append((t, pos, err, iters, conv))
        print(f"Target {i+1:2d}: [{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] -> TCP [{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] | err={err:.4f}m | iters={iters} | conv={conv}")

    print("=" * 60)

    # Visualize
    auto_exit = None
    if "--auto-exit" in sys.argv:
        auto_exit = float(sys.argv[sys.argv.index("--auto-exit") + 1])

    no_gui = "--no-gui" in sys.argv
    if no_gui:
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        for idx, (t, pos, err, iters, conv) in enumerate(results):
            # solve again to show motion
            q, _, _, _ = ik_solve(t)
            # animate
            steps = 80
            for s in range(steps):
                alpha = s / (steps - 1)
                # interpolate from current
                qcur = np.array([data.qpos[arm_qpos[j]] for j in range(n_dof)])
                qnew = (1 - alpha) * qcur + alpha * q
                for j in range(n_dof):
                    data.qpos[arm_qpos[j]] = np.clip(qnew[j], arm_ranges[j][0], arm_ranges[j][1])
                mujoco.mj_forward(model, data)
                viewer.set_texts((
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"Day 04  |  Target {idx+1}/11  |  err={np.linalg.norm(t - data.site(site_id).xpos):.4f}m  |  iters={iters}",
                    "oscillate" if idx == len(results)-1 and err > 1e-3 else ("converged" if conv else "near limit"),
                ))
                viewer.sync()
                time.sleep(1 / 40)
            time.sleep(0.8)

        viewer.set_texts((
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "Day 04  |  Position-only IK",
            "done",
        ))
        viewer.sync()
        start = time.time()
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)
            if auto_exit is not None and time.time() - start > auto_exit:
                break


if __name__ == '__main__':
    main()
