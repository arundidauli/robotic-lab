# Day 01 — Meet the arm

## Question
This robot has six joints. What does each one actually do?

## Task
1. Load the SO-101 in MuJoCo and print every joint: name, range, actuator.
2. Move one joint at a time through its full range, holding the others still.
3. Label the joint on screen while it moves.

## What you'll learn
- A robot arm is joints (degrees of freedom) plus actuators, not a brain.
- Every joint has a hard limit — the arm cannot reach everywhere you imagine.
- Simulation first: a crash costs a reload, not a servo.

## Build
| File | What it does |
|------|--------------|
| `joints.py` | Loads the scene, prints joint table, sweeps each joint with a label overlay |
| `scene_task.xml` | Copied from working sim for self-contained run |
| `so101_new_calib.xml` | SO-101 model include |
| `pick_lib.py` | Helpers copied for reuse |

```bash
cd days/day01-meet-the-arm
../../.venv/bin/mjpython joints.py
```

## Video
- **0–15 s:** six joints sweeping one by one, labelled. "This robot has six joints. Watch what each one does."
- **15–40 s:** the joint table on screen, point at the limits. "Five of these are ~180°, one is ~95° — that limit is why later days get hard."
- **40–60 s:** the full arm waving. "No AI yet. Just motors."

Caption: Six joints, six limits, zero learning. Day 1 of building a robot that teaches itself.

## Done when
- [ ] `joints.py` runs on a clean clone.
- [ ] Each joint visibly moves alone.
- [ ] Video posted and linked below.
- [ ] Issue #1 closed with video link.
- [ ] Commit: `day01: joint sweep + limits table`

Video: _link_

## Time
2 h
