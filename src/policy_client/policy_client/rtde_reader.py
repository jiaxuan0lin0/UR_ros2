"""RTDE reader for UR joint/TCP state and DH gripper register state."""

from rtde_receive import RTDEReceiveInterface

from policy_client.config import (
    DEFAULT_RTDE_FREQUENCY,
    GRIPPER_OUTPUT_REGISTER,
    UR_IP,
)
from policy_client.gripper_bridge import raw_to_norm


class RTDEReader:
    def __init__(
        self,
        robot_ip: str = UR_IP,
        frequency: float = DEFAULT_RTDE_FREQUENCY,
    ):
        self.robot_ip = robot_ip
        self.frequency = float(frequency)
        self.rtde_r = None

    def connect(self) -> None:
        self.rtde_r = RTDEReceiveInterface(
            self.robot_ip,
            frequency=self.frequency,
        )

    def disconnect(self) -> None:
        if self.rtde_r is not None:
            self.rtde_r.disconnect()
            self.rtde_r = None

    def read_state(self) -> dict:
        if self.rtde_r is None:
            raise RuntimeError("RTDE is not connected")

        joint_positions = self.rtde_r.getActualQ()
        tcp_pose = self.rtde_r.getActualTCPPose()
        raw_gripper = self.rtde_r.getOutputDoubleRegister(
            GRIPPER_OUTPUT_REGISTER
        )

        return {
            "joint_positions": joint_positions,
            "tcp_pose": tcp_pose,
            "gripper_raw": raw_gripper,
            "gripper_position": raw_to_norm(raw_gripper),
        }
