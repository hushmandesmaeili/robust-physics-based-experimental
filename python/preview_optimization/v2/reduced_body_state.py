class ReducedBodyState:
    def __init__(self, com_pos, angular_pos, com_vel, angular_vel, 
                 com_acc, angular_acc, cop, support_region, 
                 foot_pos, foot_vel, foot_acc):
        self.com_pos = com_pos
        self.angular_pos = angular_pos
        self.com_vel = com_vel
        self.angular_vel = angular_vel
        self.com_acc = com_acc
        self.angular_acc = angular_acc
        self.cop = cop
        self.support_region = support_region
        self.foot_pos = foot_pos
        self.foot_vel = foot_vel
        self.foot_acc = foot_acc