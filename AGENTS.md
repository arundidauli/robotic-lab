# AGENTS.md

## What this repo is
- Ongoing series: one task a day, tracked as GitHub issues. 30 issues per batch.
- Sim first (MuJoCo SO-101), hardware later. Real-arm tasks are in milestone "Hardware (when arm arrives)" and stay untouched until the arm is on the desk.
- Issue = source of truth until closed. Folder `days/dayNN-slug/` = proof it worked.
- Base: `https://github.com/arundidauli/robotic-lab`, board: `https://github.com/users/arundidauli/projects/3`

## Pick the next task
1. Take the lowest-numbered open issue (start at #1). Read it fully (question, task, learn, video script, done-check).
2. If it's a `[HW] ...` issue, do **not** start it yet.
3. Create `days/dayNN-slug/` by copying `days/_template/`. Name matches the issue context or keep short.
4. Implement only what the issue asks for. Cut scope if bigger than ~2–3h.

## Do the work
- Put runnable code in the day's folder. If used by >=2 days later, promote to `lib/` (don't guess now).
- Keep it reproducible from a clean clone. Put commands to run in the folder's README or the issue's task.
- Don't commit large binaries: datasets, checkpoints, raw `.mp4`, `.npz` episodes go to gitignore. Link to Hugging Face or external host. Put small gifs/stills in `media/` if needed.
- Follow the issue's video script shape (hook 0–15s, idea 15–40s, payoff 40–60s). Video linked from issue/folder, not vendored.

## Close the task cleanly
1. Update `days/dayNN-slug/README.md` with video link and any notes on what worked/broke.
2. Commit: `dayNN: <short change>`. Keep messages tied to the issue.
3. Close the issue with the video link in a comment. If it failed, say what failed and why (failure is content).
4. If you fixed a bug in shared code that affects previous days, call it out in the issue/comment.

## Repo rules
- Every day starts with a **question**.
- Every claim in a video must be reproducible from the folder.
- No datasets/checkpoints in git. Failure days get closed too with notes.
- Batches: 30 closed issues -> next 30 planned from what broke. Only one batch ahead.

## Setup / run
```bash
git clone https://github.com/arundidauli/robotic-lab.git
cd robotic-lab
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
git submodule add https://github.com/TheRobotStudio/SO-ARM100 models/SO-ARM100  # if not present
```
Deps: `mujoco==3.10.*`, `numpy`, `pyarrow`, `imageio[ffmpeg]`. Use `../.venv/bin/...` or `.venv/bin/...` depending on cwd; match the issue's command.

## Files that matter
- `README.md` — board/milestones, process
- `days/_template/` — copy for each new day
- `days/README.md` — layout
- `requirements.txt` — sim deps
- `lib/README.md`, `media/README.md` — where shared/small assets go
- Issues/milestones/project board — actual plan

## Gotchas
- Don't touch `[HW]` issues until arm arrives.
- Don't create a "10 days" or fixed course framing. This is an ongoing series in 30-day batches.
- Don't commit `.mp4`, `episodes/`, `*.npz`, `models/SO-ARM100/` — see `.gitignore` if present or follow rules above.
- Each issue may point to files/patterns from the other repo; only copy what's needed and keep it self-contained in the day's folder.
