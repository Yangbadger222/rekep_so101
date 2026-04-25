import os
import sys
import traceback
from pathlib import Path


OMNIGIBSON_ENV = Path("/home/badger/anaconda3/envs/omnigibson")
_CLEAN_ENV_FLAG = "REKEP_CLEAN_OMNIGIBSON_ENV"


def _restart_in_clean_omnigibson_env():
    """Run OmniGibson without ROS / user-site packages shadowing conda deps."""
    env = os.environ.copy()
    changed = False

    if env.get("PYTHONNOUSERSITE") != "1":
        env["PYTHONNOUSERSITE"] = "1"
        changed = True

    if env.pop("PYTHONPATH", None) is not None:
        changed = True

    ld_library_path = env.get("LD_LIBRARY_PATH")
    if ld_library_path:
        clean_ld_paths = [
            path
            for path in ld_library_path.split(os.pathsep)
            if path and not path.startswith("/opt/ros/") and "gazebo" not in path
        ]
        clean_ld_library_path = os.pathsep.join(clean_ld_paths)
        if clean_ld_library_path != ld_library_path:
            changed = True
            if clean_ld_library_path:
                env["LD_LIBRARY_PATH"] = clean_ld_library_path
            else:
                env.pop("LD_LIBRARY_PATH", None)

    target_python = OMNIGIBSON_ENV / "bin" / "python"
    if target_python.exists() and Path(sys.executable).resolve() != target_python.resolve():
        changed = True
    else:
        target_python = Path(sys.executable)

    polluted_sys_path = [
        path
        for path in sys.path
        if path.startswith("/opt/ros/")
        or path.startswith(str(Path.home() / ".local" / "lib" / "python3.10" / "site-packages"))
    ]
    if polluted_sys_path:
        changed = True

    if changed and env.get(_CLEAN_ENV_FLAG) != "1":
        env[_CLEAN_ENV_FLAG] = "1"
        os.execve(str(target_python), [str(target_python), *sys.argv], env)

    os.environ.pop("PYTHONPATH", None)
    if "LD_LIBRARY_PATH" in os.environ:
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
            path
            for path in os.environ["LD_LIBRARY_PATH"].split(os.pathsep)
            if path and not path.startswith("/opt/ros/") and "gazebo" not in path
        )
    sys.path[:] = [path for path in sys.path if path not in polluted_sys_path]


_restart_in_clean_omnigibson_env()

import numpy as np
import omnigibson as og
from omnigibson.macros import gm


def main():
    gm.HEADLESS = False
    num_steps = 150

    config = {
        "scene": {
            "type": "InteractiveTraversableScene",
            "scene_model": "Rs_int",
        },
        "robots": [
            {
                "type": "R1",
                "name": "my_robot",
            },
        ],
    }

    env = og.Environment(configs=config)
    env.reset()

    robot = env.robots[0]
    print("action dim: ", robot.action_dim)

    for i in range(num_steps):
        action = np.array([0.2, 0.0])
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        if i == 0 or (i + 1) % 10 == 0 or done:
            proprio = obs[robot.name]["proprio"]
            print(
                f"step {i + 1:03d}/{num_steps}: "
                f"reward={reward:.3f}, done={done}, "
                f"proprio[:3]={proprio[:3].cpu().numpy()}"
            )

        if done:
            break


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        sys.stdout.flush()
        sys.stderr.flush()

        # IsaacSim / OmniGibson can segfault while unloading native plugins in
        # this environment. The simulation has finished by this point, so bypass
        # Python atexit handlers and native plugin unload for this smoke test.
        os._exit(exit_code)
