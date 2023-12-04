import numpy as np
import lcm

from pydrake.examples.allegro_hand import (
    kAllegroNumJoints, AllegroHandMotionState, 
    lcmt_allegro_status, lcmt_allegro_command
)

kLcmStatusChannel = "ALLEGRO_STATUS"
kLcmCommandChannel = "ALLEGRO_COMMAND"

class PositionCommander:
    def __init__(self):
        self.lcm = lcm.LCM()
        self.lcm.subscribe(kLcmStatusChannel, self.handle_status)
        self.allegro_status = lcmt_allegro_status()
        self.allegro_command = lcmt_allegro_command()
        self.hand_state = AllegroHandMotionState()
        self.flag_moving = True

    def run(self):
        self.allegro_command.num_joints = kAllegroNumJoints
        self.allegro_command.joint_position = [0.] * kAllegroNumJoints
        self.allegro_command.num_torques = 0
        self.allegro_command.joint_torque = []

        target_joint_position = np.zeros(kAllegroNumJoints)
        self.move_to_position_until_stuck(target_joint_position)

        # Close thumb
        target_joint_position[0] = 1.396
        target_joint_position[1] = 0.3
        self.move_to_position_until_stuck(target_joint_position)

        # Close other fingers
        for i in range(4):
            start = i * 4
            target_joint_position[start:start+4] = self.hand_state.FingerGraspJointPosition(i)
        self.move_to_position_until_stuck(target_joint_position)
        print("Hand is closed.")
        while self.lcm.handle_timeout(10) == 0:
            pass

        # Record the joint position q when the fingers are close and gripping the object
        close_hand_joint_position = np.array(self.allegro_status.joint_position_measured)
        
        # Twisting the cup repeatedly
        for cycle in range(max_cycles):
            target_joint_position = close_hand_joint_position.copy()
            target_joint_position[9:12] += 0.1 * np.array([1, 1, 0.5])
            target_joint_position[0:4] = self.hand_state.FingerGraspJointPosition(0)
            target_joint_position[5:8] += 0.6 * np.array([1, 0.3, 0.5])
            self.move_to_position_until_stuck(target_joint_position)

            target_joint_position = close_hand_joint_position.copy()
            target_joint_position[9:12] += 0.1 * np.array([1, 1, 0.5])
            target_joint_position[0:4] = self.hand_state.FingerGraspJointPosition(0)
            target_joint_position[13:16] += 0.6 * np.array([1, 0.3, 0.5])
            self.move_to_position_until_stuck(target_joint_position)

    def publish_position_command(self, target_joint_position):
        self.allegro_command.joint_position = target_joint_position.tolist()
        self.lcm.publish(kLcmCommandChannel, self.allegro_command.encode())

    def move_to_position_until_stuck(self, target_joint_position):
        self.publish_position_command(target_joint_position)
        for _ in range(60):
            while self.lcm.handle_timeout(10) == 0 or self.allegro_status.utime == -1:
                pass
        while self.flag_moving:
            while self.lcm.handle_timeout(10) == 0 or self.allegro_status.utime == -1:
                pass

    def handle_status(self, channel, data):
        msg = lcmt_allegro_status.decode(data)
        self.allegro_status = msg
        self.hand_state.Update(self.allegro_status)
        self.flag_moving = not self.hand_state.IsAllFingersStuck()

if __name__ == "__main__":
    max_cycles = 1000  # You can adjust this value
    commander = PositionCommander()
    commander.run()
