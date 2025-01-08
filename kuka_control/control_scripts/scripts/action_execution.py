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
import sys, os, yaml, time
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
# IMPORT ROS2 Custom Messages:
from control_actions.msg import Action
from control_actions.msg import Joints
from control_actions.msg import Xyz
from control_actions.msg import Xyzypr

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #  

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
            
# =============================================================================== #
# ROBOT class, to execute any robot movement:

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

# Get SEQUENCE from {program}.yaml file:
def getSEQUENCE(packageNAME, yamlNAME):

    RESULT = {}

    PATH = os.path.join(get_package_share_directory(packageNAME), 'programs')
    yamlPATH = PATH + "/" + yamlNAME + ".yaml"

    if not os.path.exists(yamlPATH):
        RESULT["Success"] = False
        return(RESULT)
    
    # Get sequence from YAML:
    with open(yamlPATH, 'r') as YAML:
        seqYAML = yaml.safe_load(YAML)

    RESULT["Sequence"] = seqYAML["Sequence"]
    RESULT["Robot"] = seqYAML["Specifications"]["Robot"]
    RESULT["EEType"] = seqYAML["Specifications"]["EndEffector"]
    RESULT["EELink"] = seqYAML["Specifications"]["EELink"]
    RESULT["Objects"] = seqYAML["Specifications"]["Objects"]
    RESULT["Success"] = True
    
    return(RESULT)

# ========================================================================================= #           
# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #
def main(args=None):
    
    rclpy.init(args=args)

    # PRINT - INIT:
    print("==================================================")
    print("ROS 2 Sim-to-Real Robot Control: Program Execution")
    print("==================================================")
    print("")

    # ASSIGN -> SEQUENCE:
    SEQUENCE = [
        # {
        #     "Type": "MoveJ",
        #     "Speed": 1.0,
        #     "Input": {"joint1": 0.0, "joint2": -90.0, "joint3": -90.0, "joint4": 0.0, "joint5": 0.0, "joint6": 0.0}
        # },
        {
            "Type": "MoveRP",
            "Speed": 1.0,
            "Input": {"x": 0.4, "y": 1.5, "z": 0.8, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        },
        {
            "Type": "MoveG",
            "Speed": 1.0,
            "Input": {"value": 0.0}
        },
        {
            "Type": "MoveRP",
            "Speed": 1.0,
            "Input": {"x": 0.8, "y": 1.5, "z": 0.8, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        },
    ]

    # LOAD ROBOT/EE MOVEMENT PYTHON CLIENTS:
    print("ROBOT: ")
    RobotClient = RBT()
    print("Loaded.")
    print("")
    
    print("END-EFFECTOR:")
    print("")

    print("============================================================")
    print("============================================================")
    print("Executing sequence...")
    print("")

    # Initialise -> RES VARIABLE:
    RES = None

    # ==== EXECUTE PROGRAM, STEP BY STEP ===== #
    for x in SEQUENCE:
        try:

            print("============================================================")

            if x["Type"] == "MoveJ":
                ACTION = Action()
                ACTION.action = "MoveJ"
                ACTION.speed = x["Speed"]

                INPUT = Joints()
                INPUT.joint1 = x["Input"]["joint1"]
                INPUT.joint2 = x["Input"]["joint2"]
                INPUT.joint3 = x["Input"]["joint3"]
                INPUT.joint4 = x["Input"]["joint4"]
                INPUT.joint5 = x["Input"]["joint5"]
                INPUT.joint6 = x["Input"]["joint6"]
                ACTION.movej = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveL":
                ACTION = Action()
                ACTION.action = "MoveL"
                ACTION.speed = x["Speed"]

                INPUT = Xyz()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                ACTION.movel = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveRP":
                ACTION = Action()
                ACTION.action = "MoveRP"
                ACTION.speed = x["Speed"]

                INPUT = Xyzypr()
                INPUT.x = x["Input"]["x"]
                INPUT.y = x["Input"]["y"]
                INPUT.z = x["Input"]["z"]
                INPUT.pitch = x["Input"]["pitch"]
                INPUT.yaw = x["Input"]["yaw"]
                INPUT.roll = x["Input"]["roll"]
                ACTION.moverp = INPUT

                RES = RobotClient.Move_EXECUTE(ACTION)

            elif x["Type"] == "MoveG":
                ACTION = Action()
                ACTION.action = "MoveG"
                ACTION.speed = x["Speed"]
                ACTION.moveg = x["Input"]["value"]

                RES = RobotClient.Move_EXECUTE(ACTION)

            else:
                print("ERROR: ACTION TYPE -> " + x["Type"] + " unknown.")
                print("Closing program... BYE!")
                exit()

            # CHECK if STEP EXECUTION WAS SUCCESSFUL:
            print("")
            
            if RES["Success"] == False:
                print("ERROR: Execution FAILED!")
                print("Message -> " + RES["Message"])
                print("")
                print("Closing... BYE!")
                exit()

            else:
                print("Execution SUCCESSFUL!")
                print("Message -> " + RES["Message"])
                print("")

            time.sleep(2)
        except KeyboardInterrupt:
            
            # CANCEL ANY ONGOING ROBOT MOVEMENTS:
            RobotClient.CANCEL()

            print("Sequence execution manually interrupted and cancelled.")
            print("Closing... BYE!")
            exit()

    # ==== FINISH ===== #
    print("")
    print("")
    print("Sequence successfully executed. Closing Program... Bye!")
    print("=======================================================")

    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()