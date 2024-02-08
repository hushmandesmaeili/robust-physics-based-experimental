from typing import List
# Define ReducedBodyState class
class ReducedBodyState:
    def __init__(self, com_pos=None, com_vel=None, com_acc=None, angular_pos=None, angular_vel=None, angular_acc=None,
                 omega_R=None, omega_L=None, cop=None, rest_length=None, foot_pos=None, foot_vel=None, foot_acc=None, time=None):
        """
        Initialize the ReducedBodyState object.

        Parameters:
        com_pos (float): The position of the robot's COM, com_pos = [x, y, z].
        com_vel (float): The velocity of the robot's COM, com_vel = [x_dot, y_dot, z_dot].
        com_acc (float): The acceleration of the robot's COM, com_acc = [x_ddot, y_ddot, z_ddot].
        angular_pos (float): The orientation of the robot.
        angular_vel (float): The angular velocity of the robot.
        angular_acc (float): The angular acceleration of the robot.
        omega_R (float): The contact polygon for right foot.
        omega_L (float): The contact polygon for left foot.
        cop (float): The position of the COP.
        rest_length (float): The rest length of the robot.
        foot_pos (float): The position of the foot.
        foot_vel (float): The velocity of the foot.
        foot_acc (float): The acceleration of the foot.
        time (float): The time at which the state is recorded.
        """
        self.com_pos = com_pos
        self.com_vel = com_vel
        self.com_acc = com_acc
        self.angular_pos = angular_pos
        self.angular_vel = angular_vel
        self.angular_acc = angular_acc
        self.omega_R = omega_R
        self.omega_L = omega_L
        self.cop = cop
        self.rest_length = rest_length
        self.foot_pos = foot_pos
        self.foot_vel = foot_vel
        self.foot_acc = foot_acc
        self.time = time

class ReducedBodyTrajectory:
    def __init__(self, states: List[ReducedBodyState] = []):
        self.states = states

    def clear(self):
        self.states = []

    def add_state(self, state: ReducedBodyState):
        self.states.append(state)

    def get_state(self, index: int) -> ReducedBodyState:
        return self.states[index]

    def get_num_states(self) -> int:
        return len(self.states)

    def get_initial_state(self) -> ReducedBodyState:
        return self.states[0]

    def get_final_state(self) -> ReducedBodyState:
        return self.states[-1]

    def get_state_at_time(self, time: float) -> ReducedBodyState:
        for state in self.states:
            if state.time == time:
                return state
        return None
    
