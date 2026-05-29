# Pre-merge Checklist — `feature/fixing_object_map` → `main`

> Generated 2026-05-24. Items marked **[BLOCK]** should be fixed before merging.
> Items marked **[WARN]** are lower-priority but should be tracked.

---

## A. General / Cross-cutting

### [WARN] `import os` inside `__init__` (arbiter_node.py)
**File:** `robot_ws/src/motion_arbiter/motion_arbiter/arbiter_node.py` line 72  
`import os` is placed inside `__init__` instead of at the module top level.
Python re-evaluates it on every instantiation.  
**Fix:** move to top of file with the other imports.

### [WARN] Division-by-zero in drift correction formula (arbiter_node.py)
**File:** `robot_ws/src/motion_arbiter/motion_arbiter/arbiter_node.py`  
```python
den = -math.sin(heading_error)
s = num / den
```
The small-angle guard (`if abs(heading_error) < 0.35`) prevents the case where
`heading_error → 0`, but only for that specific branch.  If `heading_error`
is exactly `0` and `≥ 0.35` (impossible in practice, but still), `den = 0`.  
**Fix:** add `if abs(den) < 1e-9: s = 0.0` before the division.

### [WARN] Commented-out dead code in arbiter_node.py
Lines with `# self.declare_parameter("kp_wz_feedback"...)`, `# self._current_wz_measured`,
and the full `# Apply IMU-based closed-loop feedback` block are left in.  
**Fix:** remove or add a `# TODO:` tag.

---

## B. Door Mission System (from door branch)

### [BLOCK] `open_door_server._execute`: "busy" message blames debug service when it's an action overlap
**File:** `robot_ws/src/wildbot_grasp/wildbot_grasp_nodes/open_door_server.py` line 513  
When a second `open_door` goal arrives while one is running, `_motion_lock.acquire(blocking=False)`
fails and the error message says "busy: a debug motion (run_press/run_push) is running" —
but the lock is held by the previous action, not a debug service.  
**Fix:** change the error message to "busy: open_door goal already running" or use a separate
flag to distinguish action vs debug occupancy.

### [BLOCK] `open_door_server`: shutdown exit path doesn't call `goal_handle.abort()`
**File:** `robot_ws/src/wildbot_grasp/wildbot_grasp_nodes/open_door_server.py` line 655  
When `rclpy.ok()` flips to False, the `while` loop exits and `result` is returned
but `goal_handle.abort()` is never called. The goal handle is left dangling
(status = STATUS_UNKNOWN).  
**Fix:**
```python
# rclpy.ok() flipped — shutdown path.
self._publish_twist(0.0, 0.0)
result.success = False
result.message = "shutdown"
goal_handle.abort()     # ← add this
return result
```

### [WARN] `open_door_server` accepts a new goal while running (should REJECT)
**File:** `robot_ws/src/wildbot_grasp/wildbot_grasp_nodes/open_door_server.py` line 217  
`goal_callback` always returns `GoalResponse.ACCEPT`, so a second goal is accepted,
queued, then immediately aborted with a "busy" message.  
A server-side REJECT would be cleaner and avoids creating a dangling goal handle.  
**Fix:**
```python
goal_callback=lambda _r: GoalResponse.REJECT if not self._motion_lock.acquire(blocking=False)
    else (self._motion_lock.release() or GoalResponse.ACCEPT),
```
Or add a simple `_active` flag guarded by a lock.

### [WARN] `DoorMissionStatusResponse` docstring omits "driving" phase
**File:** `apps/igvi_host/models.py`  
The inline comment says `# idle | navigating | opening | succeeded | failed | canceled`
but the coordinator also emits `"driving"` (nav plan computed, motion_arbiter driving).  
The UI `_PHASE_STYLE` in `door_mission_control.py` does handle it correctly, but the
model docstring is misleading.  
**Fix:** add `driving` to the comment.

### [WARN] `DoorMissionControl.speed_spin` fires HTTP on every `valueChanged`
**File:** `apps/igvi_ui/widgets/door_mission_control.py` line 161  
```python
self.speed_spin.valueChanged.connect(self._apply_approach_speed)
```
Every spinner increment triggers an HTTP `set_params` call to `open_door_server`.
Compared to the arm sliders in `door_page.py` which use a `QTimer` debounce (70 ms),
this is inconsistent.  
**Fix:** either use `editingFinished` instead of `valueChanged`, or add a short-interval
`QTimer` debounce like the arm slider pattern.

### [WARN] Door-mission cancel in "driving" phase: coordinator emits misleading message
**File:** `robot_ws/src/igvi_bridge/igvi_bridge/door_mission.py` line 107  
When `phase == "driving"`, `cancel()` calls `_cancel_nav_goal()` which returns
"no active goal" because Nav2 already succeeded. The returned `msg` is then embedded
in the coordinator's final cancel message, producing confusing output like
`"canceled in phase 'driving': no active goal"`.  
**Fix:** skip the nav cancel call in driving phase; the bridge already handles motion
stop via `/motion/clear_path`:
```python
elif phase == "driving":
    msg = "path cleared"   # bridge already published /motion/clear_path
```

---

## C. Compose / Deployment

### [WARN] `arena_mission_server` still on `grasp` profile; others moved to `task_server`
**File:** `docker/compose/services/wildbot_grasp.yaml`  
After the profile restructuring:
- `bridge_retrieve`, `search_retrieve_server`, `open_door_server` → `task_server`
- `arena_mission_server` → still `grasp`  

This inconsistency means a `--profile task_server` start is incomplete for arena tasks.  
**Fix:** move `arena_mission_server` to `task_server`, or document the intended
profile-combination matrix explicitly in `README.md`.

### [WARN] `arm_safeguard` service removed from compose without migration note
**File:** `docker/compose/services/wildbot_grasp.yaml`  
`arm_safeguard` was a standalone service; it is now embedded in `grasp_stack`.
Anyone with a running stack will have two `arm_safeguard` nodes after updating
(the old container + the new `grasp_stack` one), causing topic collision on
`/arm_safeguard/target_trajectory`.  
**Fix:** add a migration step in `README.md`:
```bash
docker compose -f compose.yaml stop arm_safeguard
docker compose -f compose.yaml rm -f arm_safeguard
```

### [WARN] Profile matrix not documented — door mission needs two profiles
`DoorPage` calls `door_mission_start` which requires `open_door_server` (profile
`task_server`). Running `--profile robot` alone leaves the server offline; the UI
shows a "server offline" error with no guidance.  
**Fix:** add a table to `README.md` or `wildbot_grasp/README.md` mapping use-cases
to required profiles.

---

## D. Minor / Style

### [INFO] `_save_poses_yaml` button duplicated in `DoorPage`
**File:** `apps/igvi_ui/pages/door_page.py` lines 289 and 435  
The "Save to YAML" button appears in both the arm card and the push card. Both call
the same API endpoint. This is intentional (convenience) but warrants a tooltip
clarifying that both buttons save *all* params, not just the section they appear in.

### [INFO] `snapshot_health()` lock-gap between `map` and `fusion_sources`
**File:** `robot_ws/src/igvi_bridge/igvi_bridge/bridge_node.py`  
`health["map"]` is captured under `self._lock` but `fusion_sources` is appended
after the lock is released. These two fields are not atomically consistent.
Acceptable for a UI health display, but worth a comment.
