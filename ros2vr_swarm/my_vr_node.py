from typing import List
import rclpy
from rclpy.node import Node
from ros2vr_interface.msg import VRobotStates, VRobotCMD, Collision
from ubicoders_vrobots.vrobots_msgs.VROBOTS_CMDS import VROBOTS_CMDS
import numpy as np

class VirtualRobotControlNode(Node):
    def __init__(self):
        super().__init__('vrobot_publisher')
        # get system id
        self.declare_parameter('sys_id', 0)
        self.sysId = self.get_parameter('sys_id').get_parameter_value().integer_value
        
        # subscriber
        self.state_subs = self.create_subscription(
                                VRobotStates,                  # ROS2 topic type (message)
                                f'/vrobot_states_{self.sysId}',# name of the topic
                                self.controller,               # callback function
                                10                             # QoS profile
                            )
        
        # publisher
        self.cmd_pub_prefix = "/vrobot_cmd_pub_" # don't change this prefix.
        self.pub_cmd = self.create_publisher(
                                VRobotCMD,                            # ROS2 topic type (message)
                                f'{self.cmd_pub_prefix}_{self.sysId}',# name of the topic
                                10                                    # QoS profile
                            )
        
        # mission specific variables
        self.checkpoints = set()

        # PI controllers
        self.vel_pi_ctrl = PIController(kp=60, ki=2.5, error_max=100)
        self.alt_pi_ctrl = PIController(kp=0.8, ki=0.01, error_max=100)

    def controller(self, statesMsg:VRobotStates):
        # altitude control
        current_altitude = -statesMsg.lin_pos.z # z axis down is positive
        alt_setpoint = 20 # 20 meters above the ground
        error_altitude = alt_setpoint - current_altitude # error is positive if below setpoint
        velocity_setpoint = self.alt_pi_ctrl.update(error_altitude) # velocity setpoint, output of the alt - PI controller
        velocity_setpoint = np.clip(velocity_setpoint, -10, 10)

        # velocity control
        vel_error = velocity_setpoint - (- statesMsg.lin_vel.z) # error is positive if below setpoint
        throttle_sp =  self.vel_pi_ctrl.update(vel_error)
        throttle = 1200 + throttle_sp
        throttle = int(np.clip(throttle, 1200, 1600))

        #=========================
        # publish control command    
        cmdMsg = VRobotCMD()
        cmdMsg.sys_id = self.sysId
        cmdMsg.header.stamp = self.get_clock().now().to_msg()
        cmdMsg.cmd_id = VROBOTS_CMDS.SET_MR_THROTTLE
        cmdMsg.int_val = throttle
        self.pub_cmd.publish(cmdMsg)


class PIController:
    def __init__(self, kp=1, ki=1, error_max=100):
        self.kp = kp
        self.ki = ki
        self.integral = 0
        self.error_max = error_max

    def error_integrator(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.error_max, self.error_max)
    
    def update(self, error):
        self.error_integrator(error)
        return self.kp * error + self.ki * self.integral

def main():
    rclpy.init()
    node = VirtualRobotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
