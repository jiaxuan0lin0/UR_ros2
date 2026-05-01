"""RTDE reader for UR arm joints and gripper state."""

from rtde_receive import RTDEReceiveInterface
from recorder.config import UR_IP, GRIPPER_OUTPUT_REGISTER, DEFAULT_RTDE_FREQUENCY
from recorder.gripper_bridge import raw_to_norm

class RTDEReader:
    def __init__(self, robot_ip: str = UR_IP, frequency: float = DEFAULT_RTDE_FREQUENCY):
        self.robot_ip = robot_ip
        self.rtde_r = None
        self.frequency = frequency

    def connect(self):
        self.rtde_r = RTDEReceiveInterface(self.robot_ip, frequency=self.frequency)

    def disconnect(self):
        if self.rtde_r is not None:
            self.rtde_r.disconnect()
            self.rtde_r = None
    
    def read_state(self):
        if self.rtde_r is None:
            raise RuntimeError("RTDE is not connected")

        joint_positions = self.rtde_r.getActualQ()
        raw_gripper = self.rtde_r.getOutputDoubleRegister(GRIPPER_OUTPUT_REGISTER)
        gripper_position = raw_to_norm(raw_gripper)

        return {
            "joint_positions": joint_positions,
            "gripper_raw": raw_gripper,
            "gripper_position": gripper_position,
        }
