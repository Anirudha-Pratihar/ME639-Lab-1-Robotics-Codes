"""
TurtleBot3 Waffle Pi — MuJoCo Challenge 1
  b. Keyboard teleoperation
  c. Body-frame visualization (moves/rotates with the robot)
  d. Live rotation matrix readout

Controls (click the viewer window first so it has keyboard focus):
  W / S      -> forward / backward (both wheels same sign)
  A / D      -> rotate left / right in place (wheels opposite sign)
  Q / E      -> arc left / right (turn while moving forward)
  SPACE      -> stop (zero velocity command)
  R          -> print the rotation matrix + quaternion to the terminal
  ESC        -> quit
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = "waffle_pi.xml"
WHEEL_SPEED = 5.0          # rad/s commanded to each wheel
FRAME_AXIS_LEN = 0.18      # length of the drawn body-frame axes (m)

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

left_act = model.actuator("act_wheel_left").id
right_act = model.actuator("act_wheel_right").id
base_body = model.body("base").id
frame_site = model.site("body_frame").id

# current commanded wheel speeds (left, right), updated by keyboard
cmd = {"left": 0.0, "right": 0.0}


def key_callback(keycode):
    """mujoco.viewer key_callback receives a GLFW keycode (int)."""
    key = chr(keycode).lower() if 0 <= keycode < 256 else ""

    if key == "w":
        cmd["left"], cmd["right"] = -WHEEL_SPEED, -WHEEL_SPEED      # forward
    elif key == "s":
        cmd["left"], cmd["right"] = WHEEL_SPEED, WHEEL_SPEED        # backward
    elif key == "a":
        cmd["left"], cmd["right"] = WHEEL_SPEED, -WHEEL_SPEED       # rotate left (CCW)
    elif key == "d":
        cmd["left"], cmd["right"] = -WHEEL_SPEED, WHEEL_SPEED       # rotate right (CW)
    elif key == "q":
        cmd["left"], cmd["right"] = -WHEEL_SPEED * 0.4, -WHEEL_SPEED  # arc left
    elif key == "e":
        cmd["left"], cmd["right"] = -WHEEL_SPEED, -WHEEL_SPEED * 0.4  # arc right
    elif keycode == 32:  # SPACE
        cmd["left"], cmd["right"] = 0.0, 0.0
    elif key == "r":
        print_rotation_matrix()


def print_rotation_matrix():
    """Body-frame -> world-frame rotation matrix, read straight from MuJoCo's xmat."""
    R = data.xmat[base_body].reshape(3, 3)   # MuJoCo stores this for us every step
    quat = data.xquat[base_body]             # (w, x, y, z)
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    np.set_printoptions(precision=4, suppress=True)
    print("\n--- Body frame -> World frame rotation matrix R ---")
    print(R)
    print(f"quaternion (w,x,y,z): {quat}")
    print(f"yaw about world Z: {yaw:.2f} deg")
    print(f"columns of R = body axes expressed in world coords:")
    print(f"  x_body (forward) -> {R[:,0]}")
    print(f"  y_body (left)    -> {R[:,1]}")
    print(f"  z_body (up)      -> {R[:,2]}")


def draw_body_frame(viewer):
    """Draw live RGB axes at the robot's body-frame origin using the current xmat/xpos."""
    scn = viewer.user_scn
    scn.ngeom = 0

    origin = data.xpos[base_body].copy()
    R = data.xmat[base_body].reshape(3, 3)

    axis_colors = [
        (1, 0, 0, 1),   # X (forward)  -> red
        (0, 1, 0, 1),   # Y (left)     -> green
        (0, 0, 1, 1),   # Z (up)       -> blue
    ]
    for i, color in enumerate(axis_colors):
        direction = R[:, i]
        end = origin + FRAME_AXIS_LEN * direction
        g = scn.geoms[scn.ngeom]
        # must init first (sets defaults for pos/mat/size/rgba), THEN connector overrides pos/mat/size
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3), np.zeros(3), np.eye(3).flatten(),
            np.array(color, dtype=np.float32),
        )
        mujoco.mjv_connector(
            g,
            mujoco.mjtGeom.mjGEOM_ARROW,
            0.01,          # width
            origin,
            end,
        )
        scn.ngeom += 1


def main():
    print(__doc__)
    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        viewer.user_scn.ngeom = 0
        last_print = 0.0
        while viewer.is_running():
            step_start = time.time()

            data.ctrl[left_act] = cmd["left"]
            data.ctrl[right_act] = cmd["right"]

            mujoco.mj_step(model, data)
            draw_body_frame(viewer)
            viewer.sync()

            # auto-print the rotation matrix a few times a second while moving
            if time.time() - last_print > 0.5 and (cmd["left"] or cmd["right"]):
                print_rotation_matrix()
                last_print = time.time()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
