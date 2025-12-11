import rclpy
import sys
from rclpy.node import Node
import time

import cv2
import cv_bridge
import numpy as np
import os

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from omx_cpp_interface.msg import ArmGripperPosition, ArmJointAngles

# 
from rl_agent_nodes import Attack_move, Defend_move


TYPE = input("Press 1 to load defender, press 2 to load attacker ") ## input the player 
if TYPE == '1':
    TYPE = 'Defense'
else:
    TYPE = 'Attack'

class ExecuteOptimal(Node):
    def __init__(self, player):
        super().__init__('execute_optimal_policy')
        # getting shared directory to be able to access q_matrix


         # get ROS_DOMAIN_ID
        ros_domain_id = os.getenv("ROS_DOMAIN_ID")
        try:
            if int(ros_domain_id) < 10:
                ros_domain_id = "0" + str(int(ros_domain_id))
            else:
                ros_domain_id = str(int(ros_domain_id))
        except Exception:
            ros_domain_id = "00"

        self.get_logger().info(f'ROS_DOMAIN_ID: {ros_domain_id}')   

        # intializing time and state variables
        self.action_MIN_RUN_TIME = 3.5 
        self.start_time = None
        self.step = "Accessing"


         # Fetch Actions and states (saved files)
         # Joint action --> [Attack, Defender]
         # Attack Agent can Face/Turn to face opponent (0). Move_R/L (1,2), Arm home (3) Trick Move?Attack (4,5)
         # Defend Agent can turn to face (0, Turn R/L (1,2, arm_up (3) arm_right/left (4,5)
        action_pth = os.path.join(self.share, 'matrices', 'action.txt')
        self.actions = np.loadtxt(action_pth)


        # Atack agent states are indicies 0-1 and Defend are 2-3 
        # index 0 represents the Attack Agent stance relative to Defense
        # ==> 0 - in front, 1 - to the Right, 2 - to the left 
        # index 1 represens the A Agent with Arm in Disguise (1), or Attacking (2), or just home (0)
        
        # index 2 represents the Defense Agent stance relative to Attack
        # ===> forward facing (0), to_right (1), to_left (2)
        # index 3 represents the D Agent Arm position 
        # ==> home/up (0), arm_right (1), arm_left (2)
        state_pth = os.path.join(self.share, 'matrices', 'states.txt')
        self.states = np.loadtxt(state_pth)


        # matric of states on the columns and the rows. 
        # cells reutrn action indiceies if accessible, -1 if not.
        state_nstate_pth = os.path.join(self.share, 'matrices', 'state_nstate.txt')
        self.state_nstate = np.loadtxt(state_nstate_pth)


        # Saved RL matrix --- state-action-agent matrix 
        matrix_path = os.path.join(self.share, 'matrices', 'RL_matrix.txt')
        self.q_matrix = np.loadtxt(matrix_path) # np array of q_matrix 
        self.get_logger().info(f'Loaded RL_matrix')

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        #Topics 
        cmd_vel_topic = f"/tb{ros_domain_id}/cmd_vel"
        compress_image_topic = f'/tb{self.ros_domain_id}/oakd/rgb/preview/image_raw/compressed'
        scan_topic = f'/tb{self.ros_domain_id}/scan'
        arm_grip = f'/tb{ros_domain_id}/target_gripper_position'
        arm_angles = f'/tb{ros_domain_id}/target_joint_angles'


        # publishing
        self.joint_arm_pub = self.create_publisher(ArmJointAngles, arm_angles, 10)
        self.gripper_pub = self.create_publisher(ArmGripperPosition, arm_grip, 10)
        self.movement_publisher =  self.create_publisher(Twist, cmd_vel_topic, 10)


        # subscriptions 
        self.image_sub = self.create_subscription(CompressedImage, 
            compress_image_topic, 
            self.image_callback, #trigger image for robot to look around (movement)
            10
        )

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, 
            self.scan_callback,10)
        

        timer_period =  0.5
        # generating policy loop, can adjust how often function is being called
        self.timer = self.create_timer(timer_period, self.policy_loop)
        
        
        time.sleep(2)

        # Player and policy execution variables
        self.players = ["Attack", "Defend"]  # None only when it hasnt chosen a player yet
        self.player_idx = None



        self.fourht_wall_path =  os.path.join('.', 'fourth_wall.txt') # text file to communicate
        self.fourth_wall_file = np.loadtxt(self.fourht_wall_path)
        self.send_join_info()
        # Wait until 2 robots are running this file
        

        # Read file and confirm who's who--- each agent write in an action --> read action to perfomr 

        self.perform_action()



            
    def move_robot(self, action):
        None  # use implemented classes to perform movement




    def send_join_info():
        # given the current state, choose the next joint action 
        return 
    

    def policy_loop(self): # RUns every once in a while, concurrent with scan  and image_callback
        if self.step == "Moving":      # currently performing action  
            self.perform_action()


        if self.step == "Accessing":
            None 



def main(args=None):
    rclpy.init(args=args)
    node = ExecuteOptimal(TYPE) # get the node 

    # wait until both robots have written into the file ===> given the player, executre file 

    # execute policy -- check that each agent is getting their own action,

    # each robot performs their won actino 

    # until exterimination 



    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





