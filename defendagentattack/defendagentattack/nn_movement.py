import rclpy
import sys
from rclpy.node import Node
from abc import ABC, abstractmethod
import time
import os

import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from omx_cpp_interface.msg import ArmGripperPosition, ArmJointAngles


class MovementNode(Node, ABC):
    """ Superclass containing all shared attributes, initializations, and helper methods. """
    
    def __init__(self, node_name, action, pst_action):
        super().__init__(node_name)
        
        # 2. Shared Data/RL parameters
        self.curr_at = action
        self.past_act = pst_action

        # 3. Get ROS_DOMAIN_ID (Shared setup)
        ros_domain_id = os.getenv("ROS_DOMAIN_ID")
        try:
            domain_id_int = int(ros_domain_id)
            if domain_id_int < 10:
                self.ros_domain_id = "0" + str(domain_id_int)
            else:
                self.ros_domain_id = str(domain_id_int)
        except Exception:
            self.ros_domain_id = "00"

        self.get_logger().info(f'ROS_DOMAIN_ID: {self.ros_domain_id}')  

        # 4. Shared Timing/Action Variables
        self.action_MIN_RUN_TIME = 2.0  # seconds 
        self.start_time = None 
        
        # 5. Shared home pose 
        self.home_pose = [0.0, 0.0, 0.0, 0.0] # FILL THIS IN 
        self.scan_dist = None 
        self.desired_scan_dist = 0.3 
        
        # diff variables for Attack and Defend 
        self.G_R_arm_pose = []
        self.desired_angle = 0.0 
        self.angvelocity = 0.0 
        self.desired_dist = 0.0
        self.linear_v = 0.0

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # actions are 3 for defense: 
        #Guard Left, Guard Right and Reverse 

        #Topics 
        cmd_vel_topic = f"/tb{ros_domain_id}/cmd_vel"
        # compress_image_topic = f'/tb{self.ros_domain_id}/oakd/rgb/preview/image_raw/compressed'
        scan_topic = f'/tb{self.ros_domain_id}/scan'
        arm_grip = f'/tb{ros_domain_id}/target_gripper_position'
        arm_angles = f'/tb{ros_domain_id}/target_joint_angles'


        # publishing
        self.joint_arm_pub = self.create_publisher(ArmJointAngles, arm_angles, 10)
        self.gripper_pub = self.create_publisher(ArmGripperPosition, arm_grip, 10)
        self.movement_publisher =  self.create_publisher(Twist, cmd_vel_topic, 10)

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, 
            self.scan_callback,10)
        # subscriptions 
        # image to make sure you are in front of the robot
        # self.image_sub = self.create_subscription(CompressedImage, 
        #     compress_image_topic, 
        #     self.image_callback, #trigger image for robot to look around (movement)
        #     10
        # )


    def body_movement(self, neg_velo, forward=False): # if velocity is negative then it will turn left 
        x = 1
        if neg_velo:
            x = -1 
        twist_msg = Twist()
        twist_msg.angular.z = x*self.angvelocity# tbh i don't know which way is left or not 
        
        if forward: 
            twist_msg.linear.x = self.linear_v
        self.movement_publisher.publish(twist_msg)


    def arm_move(self,left_joint_1, pose):
        # if joint angle negative ==== Guarding left 
        x = -1 if left_joint_1 else 1 
        arm_msg = ArmJointAngles()
        arm_msg.joint1 = x*pose[0] 
        arm_msg.joint2, arm_msg.joint3, arm_msg.joint4 = pose[1:3]
        self.joint_arm_pub.publish(arm_msg)


    def scan_callback(self, scan:LaserScan):
        self.get_logger().debug('LaserScan received')
        scan_ranges = np.array(scan.ranges)
        num_points = len(scan_ranges)

        # Define slices for forward direction
        window = max(5, num_points // 25) 

        forward_slice = np.concatenate((scan_ranges[:window],scan_ranges[-window:]))

        valid = forward_slice[np.isfinite(forward_slice)]
        if len(valid) == 0:
            self.get_logger().warn("No valid scan points in front.")
            return

        self.scan_dist = float(np.median(valid))


    def policy_loop(self):
        self.run_action()

    @abstractmethod
    def run_action(self):
        raise NotImplementedError("run_action method must be implemented by the child class (Defense/Attack).")
        

    

class Attack_move(MovementNode):
    def __init__(self, action, pst_action):
        super().__init__('attack_move_nn', action, pst_action)

        
        # tie variables for performing actions (limit)
        self.action_MIN_RUN_TIME = 2 # seconds 
        self.start_time = None
    
        # self.step = "Accessing"
        
        self.curr_at = action
        self.past_act = pst_action 
        self.A_R_arm_pose = [0,0,0,0] # GIve this values 

        self.desired_angle = 60 ### DESIRED ANGLE IS THE ONLY THING THAT CHANGED 
        self.angvelocity = self.desired_angle/(self.action_MIN_RUN_TIME)

        self.desired_dist = 0.3 # meters 
        self.linear_v = self.desired_dist/self.action_MIN_RUN_TIME
        # THis is arm pose for R but Left is negative of the first joint 


        timer_period =  0.5
        self.timer = self.create_timer(timer_period, self.policy_loop)
    
        time.sleep(1)
        self.run_action()


    
    def run_action(self):
        
        if self.start_time != None: 
            self.start_time = self.get_clock().now()
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9


# AFTER 2 SECONDS, it should have finished the action, if not 0.3 meters away, shut down program 
        if elapsed > self.action_MIN_RUN_TIME:
            self.get_logger().info("Action should have finished performing")
            
            if (self.scan_dist - self.desired_scan_dist) < 0.1:
                rclpy.shutdown() ## immediate shutdown if the robots are very close to eachother

            self.start_time = None 


        if self.curr_at == 'A_L':

            # complete 30 dtegree movement in 2.0 seconds
            self.body_movement(neg_velo=True,forward=True)
            self.arm_move(True, self.A_R_arm_pose)

        if self.curr_at == 'A_R':
                  # complete 30 dtegree movement in 2.0 seconds 
            self.body_movement(neg_velo=False, forward=True)
            self.arm_move(False, self.A_R_arm_pose)

            
        if self.curr_at == 'R': # reversed 
            
            # with the past action reverse it, 
            # givem, self.desired angle, and home pose 

            if self.past_act == 'A_R':
                self.body_movement(neg_velo=True)
                self.arm_move(False, self.home_pose)

            else: 
                # when its G_L ==> want to riht 
                self.body_movement(neg_velo=False)
                self.arm_move(False, self.home_pose)
    


class Defense_move(MovementNode):
    def __init__(self, action, pst_action):
        super().__init__('defense_move_nn', action, pst_action)
        
        self.action_MIN_RUN_TIME = 2 # seconds 
        self.start_time = None
            
        self.curr_at = action
        self.past_act = pst_action 

        self.G_R_arm_pose = [1.33, 0.4, 0.2, 0.8]
        self.desired_angle = 30 
        self.angvelocity = self.desired_angle/(self.action_MIN_RUN_TIME)

        timer_period =  0.5
        self.timer = self.create_timer(timer_period, self.policy_loop)
        

        time.sleep(2)
        self.run_action()



    def run_action(self):
        
        if self.start_time != None: 
            self.start_time = self.get_clock().now()
            
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

     # first instance of 3.5 second run, go to the next action, after accesing state
        if elapsed > self.action_MIN_RUN_TIME:
            self.get_logger().info("Action should have finished performing")
            self.start_time = None 


        if self.curr_at == 'G_L':

            # complete 30 dtegree movement in 2.0 seconds
            self.body_movement(neg_velo=True)
            self.arm_move(True, self.G_R_arm_pose)

        if self.curr_at == 'G_R':
                  # complete 30 dtegree movement in 2.0 seconds 
            self.body_movement(neg_velo=False)
            self.arm_move(False, self.G_R_arm_pose)

            
        if self.curr_at == 'R': # reversed 

# given a known past actino and a home pose 
            if self.past_act == 'G_R':
                self.body_movement(neg_velo=True)
                self.arm_move(False, self.home_pose)

            else: 
                # when its G_L ==> want to riht 
                self.body_movement(neg_velo=False)
                self.arm_move(False, self.home_pose)

