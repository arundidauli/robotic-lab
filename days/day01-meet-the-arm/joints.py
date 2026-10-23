#!/usr/bin/env python3
import os, sys, time
import mujoco
import mujoco.viewer

MODEL = os.path.join(os.path.dirname(__file__), "scene_task.xml")


def main():
    if not os.path.exists(MODEL):
        print("Missing scene_task.xml. Put the scene here or set MODEL.")
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print("=" * 60)
    print("SO-101 joints")
    print("=" * 60)
    print(f"{'name':<25} {'type':<10} {'range':<15} {'qposadr':<10}")
    print("-" * 60)
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jtype = model.jnt_type[i]
        tname = ['free', 'ball', 'slide', 'hinge'][jtype]
        if jtype == 3:
            rng = model.jnt_range[i]
            print(f"{name or f'joint{i}':<25} {tname:<10} [{rng[0]:.3f},{rng[1]:.3f}]{'':4} {model.jnt_qposadr[i]:<10}")
        else:
            print(f"{name or f'joint{i}':<25} {tname:<10} {'-':<15} {model.jnt_qposadr[i]:<10}")
    print("-" * 60)
    print(f"nq={model.nq} nv={model.nv}")
    print()

    arm_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name and name != 'cube' and model.jnt_type[i] == 3:
            arm_joints.append((i, name))
    print(f"Arm hinge joints: {[n for _,n in arm_joints]}")
    print()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        for jidx, jname in arm_joints:
            rng = model.jnt_range[jidx]
            qadr = model.jnt_qposadr[jidx]
            print(f"Sweeping {jname} [{rng[0]:.3f} -> {rng[1]:.3f}]")
            data.qpos[qadr] = (rng[0] + rng[1]) / 2.0
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.8)
            steps = 150
            for s in range(steps):
                t = s / (steps - 1)
                data.qpos[qadr] = rng[0] + t * (rng[1] - rng[0])
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(1 / 30)
            time.sleep(0.4)
            data.qpos[qadr] = (rng[0] + rng[1]) / 2.0
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.6)
        print("Done. Close the viewer to exit.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)


if __name__ == '__main__':
    main()
