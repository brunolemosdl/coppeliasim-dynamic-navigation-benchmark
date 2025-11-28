import time
from dataclasses import dataclass
from typing import Any

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from utils.geometry import quaternion_to_euler

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

class CoppeliaSimSession:
    def __init__(
        self, host: str = "localhost", port: int = 23000, auto_connect: bool = True
    ) -> None:
        self.host: str = host
        self.port: int = port
        self.client: RemoteAPIClient | None = None
        self.simulation: Any | None = None

        if auto_connect:
            self.connect()

    def __enter__(self) -> "CoppeliaSimSession":
        if self.client is None:
            self.connect()

        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def connect(self) -> None:
        self.client = RemoteAPIClient(host=self.host, port=self.port)
        self.simulation = self.client.getObject("sim")

    def close(self) -> None:
        try:
            if self.client is not None and self.simulation is not None:
                try:
                    if self.simulation.getSimulationState() != self.simulation.simulation_stopped:
                        try:
                            self.simulation.stopSimulation()
                        except Exception:
                            pass
                except Exception:
                    pass
        finally:
            self.simulation = None
            self.client = None

    def start(self) -> None:
        self._ensure_ready()

        self.simulation.startSimulation()

    def stop(self) -> None:
        self._ensure_ready()

        self.simulation.stopSimulation()

    def step(self) -> None:
        self._ensure_ready()

        self.simulation.step()

    def is_running(self) -> bool:
        self._ensure_ready()

        return self.simulation.getSimulationState() != self.simulation.simulation_stopped

    def wait_until_running(self, timeout_seconds: float = 5.0) -> None:
        self._ensure_ready()

        start_time = time.time()

        while not self.is_running():
            if time.time() - start_time > timeout_seconds:
                raise TimeoutError("Simulation did not start within the specified timeout.")

            time.sleep(0.05)

    def wait_until_stopped(self, timeout_seconds: float = 5.0) -> None:
        self._ensure_ready()

        start_time = time.time()

        while self.is_running():
            if time.time() - start_time > timeout_seconds:
                raise TimeoutError("Simulation did not stop within the specified timeout.")

            time.sleep(0.05)

    def get_simulation_time(self) -> float:
        self._ensure_ready()

        return float(self.simulation.getSimulationTime())

    def get_object(self, path: str) -> int:
        self._ensure_ready()

        try:
            return int(self.simulation.getObject(path))
        except Exception as e:
            raise RuntimeError(f"Failed to get object '{path}': {e}") from e

    def get_pose2d_world(self, object_handle: int) -> Pose2D:
        self._ensure_ready()

        try:
            position = self.simulation.getObjectPosition(
                object_handle, self.simulation.handle_world
            )

            quaternion_x, quaternion_y, quaternion_z, quaternion_w = (
                self.simulation.getObjectQuaternion(object_handle, self.simulation.handle_world)
            )
            _, _, yaw = quaternion_to_euler(quaternion_x, quaternion_y, quaternion_z, quaternion_w)

            return Pose2D(x=float(position[0]), y=float(position[1]), theta=float(yaw))
        except Exception as e:
            raise RuntimeError(f"Failed to get pose of object '{object_handle}': {e}") from e

    def _ensure_ready(self) -> None:
        if self.client is None or self.simulation is None:
            raise RuntimeError(
                "Session is not connected. Call connect() first or use context manager: "
                "with CoppeliaSimSession() as simx:"
            )
