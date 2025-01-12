#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: October, 2024.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# System functions and classes:
import time
# Required to include ROS2 and its components:
import rclpy
from ament_index_python.packages import get_package_share_directory
import time
# Required to include ROS2 and its components:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import /Move and /RobMove ROS2 Actions:
from control_actions.action import Move
from control_actions.msg import Action
from control_actions.msg import Joints
from control_actions.msg import Xyzypr


RES_DICT = {}
RES_DICT["Success"] = False
RES_DICT["Message"] = "null"
RES_DICT["ExecTime"] = -1.0

class MoveCLIENT(Node):
    def __init__(self):

        super().__init__('ros2srrc_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print("[CLIENT - robot.py]: Initialising ROS2 /Move Action Client!")
        print("[CLIENT - robot.py]: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("[CLIENT - robot.py]: /Move ACTION SERVER detected, ready!")
        print("")

    def send_goal(self, ACTION):

        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.movej = ACTION.movej
        goal_msg.movel = ACTION.movel
        goal_msg.moverp = ACTION.moverp
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            print('[CLIENT - robot.py]: Move ACTION CALL -> GOAL has been REJECTED.')
            return
        
        print('[CLIENT - robot.py]: Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES_DICT
        
        RESULT = future.result().result  
        RES_DICT["Message"] = RESULT.result 

        if "FAILED" in RES_DICT["Message"]:
            RES_DICT["Success"] = False
        else:
            RES_DICT["Success"] = True
            

class RBT():

    def __init__(self):

        # Initialise /Move and /RobMove Action Clients:
        self.MoveClient = MoveCLIENT()
        self.EXECUTING = ""

    def Move_EXECUTE(self, ACTION):

        global RES_DICT
        self.EXECUTING = "Move"
        
        T_start = time.time()

        # Initialise RES_DICT:
        RES_DICT["Success"] = False
        RES_DICT["Message"] = "null"
        RES_DICT["ExecTime"] = -1.0
        
        self.MoveClient.send_goal(ACTION)
        while rclpy.ok():
            rclpy.spin_once(self.MoveClient)

            if (RES_DICT["Message"] != "null"):
                break

        print('[CLIENT - robot.py]: Move ACTION EXECUTED -> Result: ' + RES_DICT["Message"])
        print("")
        
        T_end = time.time()
        T = round((T_end - T_start), 4)
        RES_DICT["ExecTime"] = T

        self.EXECUTING = ""
        return(RES_DICT)
    
    def CANCEL(self):
        
        print('[CLIENT - robot.py]: MOVEMENT CANCEL REQUEST. Stopping robot...')
        
        try:
            if self.EXECUTING == "Move":
                self.MoveClient.goal_handle.cancel_goal_async()
            else:
                None
        except AttributeError:
            pass
        
        print('[CLIENT - robot.py]: MOVEMENT CANCEL REQUEST. Robot stopped.')
        print("")

class KukaActionHelper:
    def  __init__(self):
        self.action_executer = RBT()

    def move_joints(self, joint_list, speed):
        action_msg = Action()
        action_msg.action = "MoveJ"
        action_msg.speed = speed

        joint_msg = Joints()
        joint_msg.joint1 = joint_list[0]
        joint_msg.joint2 = joint_list[1]
        joint_msg.joint3 = joint_list[2]
        joint_msg.joint4 = joint_list[3]
        joint_msg.joint5 = joint_list[4]
        joint_msg.joint6 = joint_list[5]
        action_msg.movej = joint_msg

        result = self.action_executer.Move_EXECUTE(action_msg)
        return result["Success"]
    
    def move_xyzw(self, x, y, z, roll, pitch, yaw, speed=1.0):
        action_msg = Action()
        action_msg.action = "MoveRP"
        action_msg.speed = speed

        pose_msg = Xyzypr()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.z = z
        pose_msg.roll = roll
        pose_msg.pitch = pitch
        pose_msg.yaw = yaw
        action_msg.moverp = pose_msg

        result = self.action_executer.Move_EXECUTE(action_msg)
        return result["Success"]

    def toggle_gripper(self, value, speed=1.0):
        action_msg = Action()
        action_msg.action = "MoveG"
        action_msg.speed = speed
        action_msg.moveg = value

        result = self.action_executer.Move_EXECUTE(action_msg)
        return result["Success"]

def main(args=None):
    
    rclpy.init(args=args)

    action_executer = KukaActionHelper()
    action_executer.move_xyzw(0.4, 1.5, 0.8, 0.0, 0.0, 0.0)
    action_executer.toggle_gripper(0.0)

    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()
