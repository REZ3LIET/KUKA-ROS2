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
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: January, 2023.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# (TBD).

# ********** ros2_execution.py ********** #
# This .py script takes the sequence of Robot Triggers defined

# Import required libraries:
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import os
import ast
import time

# Import ACTIONS:
from control_actions.action import MoveJ
from control_actions.action import MoveL
from control_actions.action import MoveXYZW
from control_actions.action import MoveG

# Import MESSAGES:
from control_actions.msg import JointPose

# Import MultiThreadedExecutor:
from rclpy.executors import MultiThreadedExecutor

# Define GLOBAL VARIABLE -> RES:
RES = "null"

# Define CLASSES for EACH ACTION:

# 1. MoveJ:
class MoveJclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveJ_client')
        self._action_client = ActionClient(self, MoveJ, 'MoveJ')
        # 2. Wait for MoveJ server to be available:
        print ("Waiting for MoveJ action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveJ ACTION SERVER detected.")
    
    def send_goal(self, GoalJP, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveJ.Goal()
        goal_msg.goal = GoalJP
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveJ ACTION CALL finished.") 

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveJ ACTION CALL.

# 2. MoveL:
class MoveLclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveL_client')
        self._action_client = ActionClient(self, MoveL, 'MoveL')
        # 2. Wait for MoveL server to be available:
        print ("Waiting for MoveL action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveL ACTION SERVER detected.")
    
    def send_goal(self, GoalLx, GoalLy, GoalLz, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveL.Goal()
        goal_msg.movex = GoalLx
        goal_msg.movey = GoalLy
        goal_msg.movez = GoalLz
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveL ACTION CALL finished.")       

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveL ACTION CALL.

# 3. MoveXYZW:
class MoveXYZWclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')
        # 2. Wait for MoveXYZW server to be available:
        print ("Waiting for MoveXYZW action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZW ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZWx, GoalXYZWy, GoalXYZWz, GoalXYZWyaw, GoalXYZWpitch, GoalXYZWroll, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = GoalXYZWx
        goal_msg.positiony = GoalXYZWy
        goal_msg.positionz = GoalXYZWz
        goal_msg.yaw = GoalXYZWyaw
        goal_msg.pitch = GoalXYZWpitch
        goal_msg.roll = GoalXYZWroll
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZW ACTION CALL finished.")     

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZW ACTION CALL.

# 4. MoveG:
class MoveGclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveG_client')
        self._action_client = ActionClient(self, MoveG, 'MoveG')
        # 2. Wait for MoveG server to be available:
        print ("Waiting for MoveG action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveG ACTION SERVER detected.")
    
    def send_goal(self, GP):
        # 1. Assign variables:
        goal_msg = MoveG.Goal()
        goal_msg.goal = GP
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveG ACTION CALL finished.")

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveG ACTION CALL.

# ==================================================================================================================================== #
# ==================================================================================================================================== #
# =============================================================== MAIN =============================================================== #
# ==================================================================================================================================== #
# ==================================================================================================================================== #

def main(args=None):
    
    # Import global variable RES:
    global RES
    
    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    MoveL_CLIENT = MoveLclient()
    MoveJ_CLIENT = MoveJclient()
    MoveG_CLIENT = MoveGclient()
    MoveXYZW_CLIENT = MoveXYZWclient()

    # Create NODE for LOGGING:
    nodeLOG = rclpy.create_node('node_LOG')
    seq = [
        {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0},
        {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': -90.0, 'joint3': 90.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0},
        {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': -90.0, 'joint3': -90.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0},
        {'action': 'MoveXYZW', 'value': {'positionx': 0.4, 'positiony': 1.5, 'positionz': 0.8, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
    ]

    # Log number of steps:
    nodeLOG.get_logger().info("Number of steps -> " + str(len(seq)))
    time.sleep(1)

    # ================================= 4. SEQUENCE ================================= #
    for i in range(0, len(seq)):
        trigger = seq[i]
        
        if (trigger['action'] == 'MoveJ'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveJ:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            JP = JointPose()
            JP.joint1 = trigger['value']['joint1']
            JP.joint2 = trigger['value']['joint2']
            JP.joint3 = trigger['value']['joint3']
            JP.joint4 = trigger['value']['joint4']
            JP.joint5 = trigger['value']['joint5']
            JP.joint6 = trigger['value']['joint6']
            MoveJ_CLIENT.send_goal(JP, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveJ_CLIENT)
                if (RES != "null"):
                    break

            print ("Result of MoveJ ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveJ:SUCCESS"):
                print("MoveJ ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveJ ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveJ ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'MoveL'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveL:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            MoveX = trigger['value']['movex']
            MoveY = trigger['value']['movey']
            MoveZ = trigger['value']['movez']
            MoveL_CLIENT.send_goal(MoveX,MoveY,MoveZ, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveL_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveL ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveL:SUCCESS"):
                print("MoveL ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveL ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveL ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'MoveXYZW'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveXYZW:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            
            MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZW_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZW ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZW:SUCCESS"):
                print("MoveXYZW ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'GripperOpen'):
            print("")
            print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

            GP = 0.7
            MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveG ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        elif (trigger['action'] == 'GripperClose'):                
            print("")
            print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

            GP = 0.0
            MoveG_CLIENT.send_goal(GP)
            
            while rclpy.ok():
                rclpy.spin_once(MoveG_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveG:SUCCESS"):
                print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveG ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        else:
            print("Step number " + str(i) + " -> Action type not identified. Please check.")
            print("The program will be closed. Bye!")
            nodeLOG.get_logger().info("ERROR: Program finished since ACTION NAME in step number -> " + str(i) + " was not identified.")
            break

        #time.sleep(1)

    print("")
    print("SEQUENCE EXECUTION FINISHED!")
    print("Program will be closed. Bye!")
    nodeLOG.get_logger().info("SUCESS: Program execution sucessfully finished.")
    nodeLOG.destroy_node()
    print("Closing... BYE!")
        

if __name__ == '__main__':
    main()