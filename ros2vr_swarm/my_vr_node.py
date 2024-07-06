from typing import List
import rclpy
from rclpy.node import Node
from ros2vr_interface.msg import VRobotStates, VRobotCMD, Collision
from ubicoders_vrobots.vrobots_msgs.VROBOTS_CMDS import VROBOTS_CMDS

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

    def controller(self, statesMsg:VRobotStates):
        # for collision checking        
        colls:List[Collision] = statesMsg.collisions
        for (i, coll) in enumerate(colls):
            if (coll.collision_type ==1):
                obj_name = coll.object_name
                self.checkpoints.add(obj_name)
        self._logger.info(f"Checkpoints: {self.checkpoints}")

        #=========================
        # Implement some cool control logic


        #=========================

        # Example: set all motors to 1510
        throttle = 1510
        final_pwm = [throttle, throttle, throttle, throttle]

        # publish control command    
        cmdMsg = VRobotCMD()
        cmdMsg.sys_id = self.sysId
        cmdMsg.header.stamp = self.get_clock().now().to_msg()
        cmdMsg.cmd_id = VROBOTS_CMDS.SET_MR_THROTTLE
        cmdMsg.int_val = throttle
        self.pub_cmd.publish(cmdMsg)

def main():
    rclpy.init()
    node = VirtualRobotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
