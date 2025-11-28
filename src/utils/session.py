import os
import platform
import socket
from pathlib import Path

from services.coppelia import CoppeliaSimSession
from utils.logger import get_logger

logger = get_logger("utils.session")

_COPPELIA_SESSION = None
_PATH_CONVERSION_CACHE = {}

def _is_wsl() -> bool:
    try:
        with open("/proc/version") as f:
            version = f.read().lower()
            return "microsoft" in version or "wsl" in version
    except (FileNotFoundError, PermissionError):
        return False

def _is_windows_host(host: str) -> bool:
    if host in ("localhost", "127.0.0.1"):
        return platform.system() == "Windows"

    env_host_os = os.getenv("COPPELIA_HOST_OS")
    if env_host_os:
        return env_host_os.lower() == "windows"

    if host.startswith("172.") or host.startswith("192.168.") or host.startswith("10."):
        return True

    return False

def _needs_path_conversion(host: str) -> bool:
    env_override = os.getenv("COPPELIA_PATH_CONVERSION")
    if env_override is not None:
        return env_override.lower() in ("1", "true", "yes", "on")

    if not _is_wsl():
        return False

    return _is_windows_host(host)

def _convert_path_for_coppelia(path: str, host: str) -> str:
    if path in _PATH_CONVERSION_CACHE:
        return _PATH_CONVERSION_CACHE[path]

    if not _needs_path_conversion(host):
        _PATH_CONVERSION_CACHE[path] = path
        return path

    path_obj = Path(path).resolve()
    path_str = str(path_obj)

    if path_str.startswith("/mnt/"):
        parts = path_str.split("/")
        if len(parts) >= 3:
            drive = parts[2].upper()
            rest = "/".join(parts[3:])
            backslash = "\\"
            windows_path = f"{drive}:{backslash}{rest.replace('/', backslash)}"
            _PATH_CONVERSION_CACHE[path] = windows_path
            return windows_path

    _PATH_CONVERSION_CACHE[path] = path_str
    return path_str

def get_coppelia_session(host="localhost", port=23000):
    global _COPPELIA_SESSION
    if _COPPELIA_SESSION is None:
        _COPPELIA_SESSION = CoppeliaSimSession(host=host, port=port)
    return _COPPELIA_SESSION

def reset_coppelia_session():
    global _COPPELIA_SESSION
    if _COPPELIA_SESSION:
        _COPPELIA_SESSION.close()
    _COPPELIA_SESSION = None
    _PATH_CONVERSION_CACHE.clear()

def load_scene(scene_path: str) -> None:
    session = get_coppelia_session()
    sim = session.simulation
    host = session.host

    session.start()
    session.wait_until_running()

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            session.step()

    coppelia_path = _convert_path_for_coppelia(scene_path, host)
    sim.loadScene(coppelia_path)

    session.stop()
    session.wait_until_stopped()

def check_connection(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False

def get_goal_position(
    session: CoppeliaSimSession, goal_name: str = "/Goal"
) -> tuple[float, float] | None:
    try:
        simulation = session.simulation
        goal_handle = simulation.getObject(goal_name)

        position = simulation.getObjectPosition(goal_handle, simulation.handle_world)

        goal_x = float(position[0])
        goal_y = float(position[1])

        return (goal_x, goal_y)

    except Exception:
        alternative_names = ["/goal", "/Goal", "/target", "/Target"]
        for alt_name in alternative_names:
            try:
                goal_handle = session.simulation.getObject(alt_name)
                position = session.simulation.getObjectPosition(
                    goal_handle, session.simulation.handle_world
                )
                goal_x = float(position[0])
                goal_y = float(position[1])
                return (goal_x, goal_y)
            except Exception:
                continue

        logger.error("Could not find goal object in scene")
        return None

def get_other_robots(session: CoppeliaSimSession, robot_names: list | None = None) -> list:
    other_robots = []

    if robot_names is None:
        other_robot_names_env = os.getenv("OTHER_ROBOT_NAMES", "/PioneerP3DX")
        robot_names = [name.strip() for name in other_robot_names_env.split(",")]

    for robot_name in robot_names:
        try:
            robot_handle = session.simulation.getObject(robot_name)
            pose = session.get_pose2d_world(robot_handle)
            other_robots.append(pose)
        except Exception:
            continue

    return other_robots
