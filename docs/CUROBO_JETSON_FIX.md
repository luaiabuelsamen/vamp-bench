# cuRobo on jetson3: what broke and the fix

**Cause.** `~/projects/curobo_berkeley/curobo` was originally cloned at `ebb7170` (v0.7.7-5, 2025-08-04), the cuRobo v1 API the paper used. A later `git pull --ff-only origin main` fast-forwarded it to `ac5f931`, which is the cuRobo v2 rewrite (`curobo/_src/...`, a different API). That removed every v1 `.py` file under `src/curobo/`; only the untracked compiled CUDA extensions (`src/curobo/curobolib/*.so`, built 2025-09-02) remained. The venv's editable install (`nvidia_curobo 0.0.0`) also points at `/home/jetson3/ros2_ws/curobo`, which no longer exists.

**Fix (non-destructive, does not touch the frozen repo).**
```bash
git clone ~/projects/curobo_berkeley/curobo third_party/curobo_v1
git -C third_party/curobo_v1 checkout ebb7170
cp ~/projects/curobo_berkeley/curobo/src/curobo/curobolib/*.so third_party/curobo_v1/src/curobo/curobolib/
PYTHONPATH=$PWD/third_party/curobo_v1/src ~/projects/curobo_berkeley/venv/bin/python ...
```
The prebuilt extensions load against the venv's torch `2.5.0a0+872d972e41.nv24.08` (CUDA available). Smoke test: UR5e + `collision_table.yml`, `plan_single` succeeds.
