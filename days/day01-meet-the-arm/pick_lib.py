"""SO-101 pick-bucket helpers: offline kinematic IK + servo-target execution."""
import numpy as np
import mujoco

MJCF = "scene_task.xml"
SITE = "gripperframe"
SITE_NP = mujoco.mjtObj.mjOBJ_SITE
BODY_NP = mujoco.mjtObj.mjOBJ_BODY
ARM_DOFS = [0, 1, 2, 3, 4]          # dof columns for IK; gripper is dof 5
DOWN = np.array([0.0, 0.0, -1.0])


def id_of(m, kind, name):
    return mujoco.mj_name2id(m, kind, name)


class Sim:
    def __init__(self, path=MJCF):
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.sid = id_of(self.m, SITE_NP, SITE)
        self.cube_bid = id_of(self.m, BODY_NP, "cube")
        self.bucket_bid = id_of(self.m, BODY_NP, "bucket")
        self.jnt_range = self.m.jnt_range[:5].copy()   # arm joint limits (rad)
        self.sync = lambda: None                 # replaced when a viewer attaches

    @staticmethod
    def _site_axis_z(d, sid):
        x = d.site_xmat[sid]     # flattened, row-major
        return np.array([x[2], x[5], x[8]])      # site z-axis in world

    # ---------- primitives ----------
    def fwd(self):
        mujoco.mj_forward(self.m, self.d)

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)
        self.sync()

    def settle(self, secs=0.4):
        self.step(int(secs * 500))

    def reset(self, ctrl=None):
        mujoco.mj_resetData(self.m, self.d)
        if ctrl is not None:
            self.d.ctrl[:6] = ctrl
        self.fwd()
        self.settle(0.4)

    # ---------- readouts ----------
    def site_pos(self):
        return self.d.site_xpos[self.sid].copy()

    def site_zaxis(self):
        x = self.d.site_xmat[self.sid]
        return np.array([x[2], x[5], x[8]])      # site z-axis in world

    def cube_pos(self):
        return self.d.xpos[self.cube_bid].copy()

    def set_cube(self, pos=None, quat=None):
        if pos is not None:
            self.d.qpos[6:9] = pos            # arm = 6 hinges, then freejoint
        if quat is not None:
            self.d.qpos[9:13] = quat
        self.d.qvel[6:12] = 0.0
        self.fwd()

    def set_cube_yaw(self, yaw_deg):
        """rotate the free cube about world z (to align faces with the jaws)."""
        q = np.zeros(4)
        mujoco.mju_axisAngle2Quat(
            np.asarray(q, dtype=np.float64),
            np.array([0.0, 0.0, 1.0]),
            np.radians(yaw_deg),
        )
        self.set_cube(quat=q)

    def set_cube_yaw(self, yaw_deg):
        """rotate the free cube about world z (to align faces with the jaws)."""
        q = np.zeros(4)
        mujoco.mju_axisAngle2Quat(
            np.asarray(q, dtype=np.float64),
            np.array([0.0, 0.0, 1.0]),
            np.radians(yaw_deg),
        )
        self.d.qpos[9:13] = q
        self.d.qvel[6:12] = 0.0
        self.fwd()

    def touching_cube(self):
        """sorted body names currently touching the cube."""
        out = set()
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1, b2 = int(self.m.geom_bodyid[g1]), int(self.m.geom_bodyid[g2])
            if b1 == self.cube_bid:
                out.add(self.m.body(b2).name)
            if b2 == self.cube_bid:
                out.add(self.m.body(b1).name)
        return sorted(out)

    def count_gripper_touch(self):
        """how many distinct gripper bodies touch the cube (0 = none)."""
        out = set()
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1, b2 = int(self.m.geom_bodyid[int(c.geom1)]), int(self.m.geom_bodyid[int(c.geom2)])
            if self.cube_bid in (b1, b2):
                other = b1 if b2 == self.cube_bid else b2
                name = self.m.body(other).name
                if name != "world":
                    out.add(name)
        return len(out)

    def drop_error(self, bucket_center, z_below=0.05):
        p = self.cube_pos()
        return float(np.linalg.norm(p[:2] - bucket_center[:2])), float(p[2])

    # ---------- offline IK ----------
    def ik(self, target, q0, axis=DOWN, iters=300, lam=0.03, ptol=0.002, atol=0.03):
        """pure-kinematics damped least squares; returns q (6,) with gripper slot."""
        d = self.d
        q0 = np.asarray(q0, float)
        if q0.size == 5:
            q0 = np.append(q0, 0.0)      # accept gripper-less seed
        d.qpos[:6] = q0
        tgt = np.asarray(target, float)
        to = None if axis is None else np.asarray(axis, float)
        ax = np.zeros(3)
        e_p = e_r = np.zeros(3)
        for _ in range(iters):
            mujoco.mj_forward(self.m, d)
            if to is not None:
                x = d.site_xmat[self.sid]
                ax[:] = (x[2], x[5], x[8])
            e_p = tgt - d.site_xpos[self.sid]
            e_r = np.zeros(3) if to is None else _axis_err(ax, to)
            self.ik_pos_err = float(np.linalg.norm(e_p))
            self.ik_rot_err = float(np.linalg.norm(e_r))
            if self.ik_pos_err < ptol and self.ik_rot_err < atol:
                break
            Jp = np.zeros((3, self.m.nv))
            Jr = np.zeros((3, self.m.nv))
            mujoco.mj_jacSite(self.m, d, Jp, Jr, self.sid)
            if to is None:
                J = Jp[:, ARM_DOFS]
                err = e_p
            else:
                J = np.vstack([Jp[:, ARM_DOFS], Jr[:, ARM_DOFS]])
                err = np.concatenate([e_p, e_r])
            dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(err.size), err)
            dq = np.append(dq, 0.0)          # gripper dof is not part of the IK solve
            d.qpos[:6] += np.clip(dq, -0.3, 0.3)
            d.qpos[:5] = np.clip(d.qpos[:5], self.jnt_range[:, 0], self.jnt_range[:, 1])
        return d.qpos[:6].copy()

    # ---------- execution on servo targets ----------
    def exec_ctrl(self, target6, secs=0.6, settle=0.15):
        """cosine-glide the six ctrl targets toward target6 (radians, incl gripper)."""
        m, d = self.m, self.d
        t0 = d.ctrl[:6].copy()
        tgt = np.asarray(target6, float)
        n = max(1, int(secs * 400))
        for i in range(n):
            a = 0.5 - 0.5 * np.cos(np.pi * (i + 1) / n)
            d.ctrl[:6] = t0 + (tgt - t0) * a
            mujoco.mj_step(m, d)
        self.sync()
        d.ctrl[:6] = tgt
        self.settle(settle)

    def ik_line(self, p_from, p_to, n=10, axis=DOWN, q_start=None):
        """offline IK sample along a straight cartesian line; returns list of q."""
        q = (self.d.qpos[:6].copy() if q_start is None else np.asarray(q_start, float))
        pts = []
        for i in range(1, n + 1):
            t = i / n
            q = self.ik(p_from + (p_to - p_from) * t, q, axis=axis)
            pts.append(q.copy())
        return pts

    def goto(self, p_to, secs=0.6, axis=DOWN, grip=None, q_seed=None, line_from=None):
        """solve IK to p_to, glide there (optionally cart-interpolating from line_from)."""
        seed = self.d.qpos[:6].copy() if q_seed is None else q_seed
        q_end = self.ik(p_to, seed, axis=axis)
        g = self.d.ctrl[5] if grip is None else grip
        if line_from is not None:
            qs = self.ik_line(np.asarray(line_from, float), np.asarray(p_to, float),
                              axis=axis, q_start=seed)
            per = secs / max(1, len(qs))
            for q in qs:
                self.exec_ctrl(np.append(q[:5], g), secs=per)
        else:
            self.exec_ctrl(np.append(q_end[:5], g), secs=secs)
        return q_end


def _axis_err(ax, to):
    c = np.cross(ax, to)
    if float(np.dot(ax, to)) < 0.0 and float(np.linalg.norm(c)) < 1e-6:
        k = int(np.argmin(np.abs(ax)))
        perp = np.zeros(3)
        perp[k] = 1.0
        return np.cross(ax, perp)
    return c
