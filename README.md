[README.md](https://github.com/user-attachments/files/33186595/README.md)
# MuJoCo Challenge 1 — TurtleBot3 Waffle Pi

## Files
- `waffle_pi.xml` — MJCF model: differential-drive base (radius 0.14 m), two
  driven wheels (radius 0.033 m, separation 0.287 m — real Waffle Pi specs),
  a passive ball caster, and a `body_frame` site at the robot's center.
- `teleop.py` — interactive viewer: keyboard control, live body-frame axes,
  rotation matrix printout.

## Setup
```bash
pip install mujoco
cd <this folder>
python3 teleop.py
```
A window opens showing the robot on a checkered floor. **Click inside the
viewer window first** so it captures keyboard input.

## a. Spawning the robot
`waffle_pi.xml` uses a `<freejoint>` on the base body, so the robot is fully
free in 3D (not constrained to a plane) — exactly how a real robot's base
behaves before you add any control. Position/orientation live in
`data.qpos[0:7]` = `(x, y, z, qw, qx, qy, qz)`.

## b. Keyboard control
`teleop.py` registers `key_callback` with `mujoco.viewer.launch_passive`.
Each key sets a target angular velocity (rad/s) for the two wheel
`velocity` actuators — MuJoCo's built-in PD velocity servo, so you don't
need to hand-write a controller:

| Key | Left wheel | Right wheel | Motion |
|---|---|---|---|
| W | −5 | −5 | forward |
| S | +5 | +5 | backward |
| A | +5 | −5 | rotate left (CCW) |
| D | −5 | +5 | rotate right (CW) |
| Q | −2 | −5 | arc left |
| E | −5 | −2 | arc right |
| Space | 0 | 0 | stop |
| R | — | — | print rotation matrix now |

This is standard differential-drive kinematics: same-sign wheel speeds
→ straight line, opposite-sign → pure rotation about the robot's own
center, because the two contact points move in opposite directions.

## c. Body-frame visualization
`draw_body_frame()` runs every render frame. It reads MuJoCo's own
kinematics output — `data.xpos[base_body]` (world position) and
`data.xmat[base_body]` (3×3 world orientation, flattened) — and draws
three arrows from the robot's origin along its **own** X (red), Y (green),
Z (blue) axes using `mjv_connector` on `viewer.user_scn` (a scratch scene
MuJoCo lets you draw into without touching the model). Because it's
recomputed every step from `xmat`/`xpos`, the frame visibly rotates and
translates with the robot in real time as you drive it — that's the whole
point of part (c): showing the *body* frame is not fixed to the world.

## d. Rotation matrix
`print_rotation_matrix()` reads `data.xmat[base_body]`, MuJoCo's
already-computed rotation matrix (body frame → world frame), reshaped to
3×3. Two things worth understanding for your write-up:

1. **Columns = body axes expressed in world coordinates.** Column 0 is
   where the robot's local +X (forward) currently points in the world;
   column 1 is local +Y (left); column 2 is local +Z (up). That's *why*
   this matrix is what you use to draw the frame in part (c) — same data,
   two views.
2. **It's a view, not a snapshot.** `data.xmat[id].reshape(3,3)` returns a
   view into MuJoCo's internal buffer. If you store that array and then
   call `mj_step` again, the *stored* array's values will silently change
   too — always `.copy()` it if you need the value at a specific instant
   (the script already does this correctly; worth demonstrating you know
   why in your report).

The script auto-prints R a few times a second while you're driving, and
`R` on demand.

## Suggested report structure
1. Screenshot of the spawned robot + axes at rest, with R = identity-ish.
2. Screenshot mid-turn, with R and the visible axis rotation side by side.
3. Explain, using the column argument above, how R's columns match what
   you see in the viewport.
4. (Optional extension) Log `(x, y, yaw)` over time to a CSV and plot the
   trajectory — turns this into an odometry demo too.
