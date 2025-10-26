"""Universal MuJoCo MJCF viewer script.

Usage (PowerShell):
  python .\tools\view_scene.py .\RobodogV1\scene_position_robodog.xml

It tries to use the modern `mujoco` Python package first, falling back to `mujoco_py` if available.
"""
import sys
import time

XML_PATH_HELP = "Usage: python tools\\view_scene.py path/to/scene.xml"


def try_modern_mujoco(path):
    try:
        import mujoco
        from mujoco.viewer import launch
    except Exception as e:
        return False, str(e)

    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    # If the MJCF contains a keyframe, use it to set the initial qpos/ctrl so
    # the viewer shows the 'home' pose defined in <keyframe>.
    try:
        if getattr(model, 'nkey', 0) > 0:
            # model.key_qpos shape: (nkey, model.nq)
            data.qpos[:] = model.key_qpos[0]
            # model.key_ctrl shape: (nkey, model.nu) - set control targets if present
            if getattr(model, 'nkey', 0) > 0 and model.key_ctrl.size:
                data.ctrl[:] = model.key_ctrl[0]
            # forward to update derived quantities
            mujoco.mj_forward(model, data)
    except Exception:
        # Non-fatal: continue with viewer using default pose
        pass
    print("Launching modern mujoco viewer...")
    launch(model, data)
    return True, None


def try_mujoco_py(path):
    try:
        import mujoco_py
    except Exception as e:
        return False, str(e)

    model = mujoco_py.load_model_from_path(path)
    sim = mujoco_py.MjSim(model)
    # Apply keyframe pose if available so the viewer spawns the robot in the keyframe
    try:
        if hasattr(model, 'nkey') and model.nkey > 0:
            # model.key_qpos is a flat array or nested list; use first key
            try:
                sim.data.qpos[:] = model.key_qpos[0]
            except Exception:
                # some mujoco-py versions store key_qpos in different shape
                try:
                    sim.data.qpos[:] = model.key_qpos
                except Exception:
                    pass
            try:
                sim.data.ctrl[:] = model.key_ctrl[0]
            except Exception:
                pass
            sim.forward()
    except Exception:
        pass
    viewer = mujoco_py.MjViewer(sim)
    print("Launching mujoco-py viewer...")
    try:
        while True:
            sim.step()
            viewer.render()
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Exiting viewer")
    return True, None


def main():
    if len(sys.argv) < 2:
        print(XML_PATH_HELP)
        sys.exit(1)

    path = sys.argv[1]

    ok, err = try_modern_mujoco(path)
    if ok:
        return

    print("modern mujoco not available:", err)

    ok, err = try_mujoco_py(path)
    if ok:
        return

    print("mujoco-py not available either:", err)
    print("Install one of: pip install mujoco (recommended) OR pip install mujoco-py")


if __name__ == '__main__':
    main()
