#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from moveit_client import TurtlebotArmClient
import time
import ast


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        # Change this to the desired marker ID from pick_n_place.launch.py file, Declare parameter with default integer value
        self.markerid = self.declare_parameter('markerid', 1).get_parameter_value().integer_value

        self.target_marker_id = self.markerid 

        self.mark_subscription = self.create_subscription(
            MarkerArray,
            '/detected_markers',
            self.aruco_listener_callback,
            1)

        self.mission_publisher = self.create_publisher(Bool,'/mission_state',1)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

        self.twist = Twist()
        self.finish_move = False

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )


        self.get_joint = False
        self.marker = []

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()

        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    

        self.point = JointTrajectoryPoint()
        self.point.velocities = [0.0] * 4
        self.point.accelerations = [0.0] * 4
        self.point.time_from_start.sec = 0
        self.point.time_from_start.nanosec = 500
        self.point.positions = [0.0, -0.488692, -0.453786, 2.007129] 

        self.trajectory_msg.points = [self.point]
        self.joint_pub.publish(self.trajectory_msg)

        #sample_pkg/src/simple_manager_node.py
        # 상태 변수
        self.aruco_marker_found = False
        self.task_completed = False
        self.armrun = False
        self.aruco_location_x = 0
        self.aruco_location_y = 0
        self.aruco_location_z = 0       

        self.marker_id = None
        self.state = 'START'  

        self.count = 0
        self.aruco_pose = None  # Aruco marker의 pose 정보를 저장할 변수

        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')
        #self.create_timer(1.0, self.run_tasks)
        
        #self.twist = Twist()


    def joint_states_callback(self, msg):

        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    

    def aruco_listener_callback(self, msg):
        
        #Test...
        print('aruco_listener_callback')
        #self.aruco_arm_controll()
        print(f'msg : {msg}')
        self.publish_cmd_vel(0.02)
        # self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                # self.get_logger().info(f'Position: x:[{marker.pose.pose.position.x}, y:[{marker.pose.pose.position.y},z:[{marker.pose.pose.position.z}]] ')
                # self.get_logger().info(f'Orientation: x:[{marker.pose.pose.orientation.x}, y:[{marker.pose.pose.orientation.y},z:[{marker.pose.pose.orientation.z}]] ')
        # self.point.positions = [0.0, -0.488692 , -0.53786, 2.007129] 
        # self.trajectory_msg.points = [self.point]
        # self.joint_pub.publish(self.trajectory_msg)  
        for marker in msg.markers:
            if marker.id == self.target_marker_id:
                self.aruco_position_x = marker.pose.pose.position.x
                self.aruco_position_y = marker.pose.pose.position.y               
                self.aruco_position_z = marker.pose.pose.position.z
                print(marker.pose.pose.position.y)
                if marker.pose.pose.position.y > -0.02:
                    self.publish_cmd_vel(0.0)
                    self.get_logger().info(f'finish_move = True')                    
                    self.finish_move = True
                    self.aruco_arm_controll()
                elif marker.pose.pose.position.y < -0.02 :
                    self.get_logger().info(f'publish_cmd_vel(0.02)')                    
                    self.publish_cmd_vel(0.02)
    

                # # break

    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  


##################################################################################            
##################################################################################
    def append_pose_init(self, x,y,z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z
        pose_array.poses.append(pose)
        
        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array

    def aruco_arm_controll(self):

        print("Impossible Mission Start")        
        arm_client = TurtlebotArmClient()

        print(f"Mission Aruco marker Locaion coordinates: {self.aruco_location_x}, {self.aruco_location_y}, {self.aruco_location_z}")

        ##### TEST, if I cound aruco_marker...
        self.aruco_marker_found = True

        if self.aruco_marker_found:
            self.armrun = True

            self.aruco_location_x = -0.92
            self.aruco_location_y = -0.45
            self.aruco_location_z = 0.33

            print(f"Remove Rock Initial Position")
            time.sleep(2)            
            self.point.positions = [0.0, -0.488692, -0.453786, 2.007129]
            print("point",self.point.positions)
            self.joint_pub.publish(self.trajectory_msg)


            print("Move Aruco Cube(Rock) Mission")
     
            response = arm_client.send_request(1, "01_home") # move to side
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Gripper Open")
    
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')


            # print("Cube Position Start")
            # pose_array = self.append_pose_init(0.137496 - self.aruco_location_y + 0.05,0.00 - self.aruco_location_x ,0.122354 )
            # response = arm_client.send_request(0, "", pose_array)
            # arm_client.get_logger().info(f'Response: {response.response}')

            print("Cube Box Front Start...")       
            time.sleep(2)
            response = arm_client.send_request(1, "02_box_front")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Move to Box Start...")       
            time.sleep(2)                 
            response = arm_client.send_request(1, "03_move_to_box")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(2)
            print("Gripper Close")
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')



            print("Move up Start...")       
            time.sleep(2)                 
            response = arm_client.send_request(1, "04_move_up")
            arm_client.get_logger().info(f'Response: {response.response}')



            print("Conveyor up Start...")       
            time.sleep(2)  
            response = arm_client.send_request(1, "05_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor down Start...")       
            time.sleep(2)  
            response = arm_client.send_request(1, "06_conveyor_down")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(2)
            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor up Start...")       
            time.sleep(2)  
            response = arm_client.send_request(1, "07_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')


   
            response = arm_client.send_request(1, "08_home")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            #print ("Remove Rock Mission Start")

            #response = arm_client.send_request(1, "conveyor_up")
            #arm_client.get_logger().info(f'Response: {response.response}')


            #response = arm_client.send_request(2, "open")
            #arm_client.get_logger().info(f'Response: {response.response}')


            #print("Return Sweet Home ")

            #response = arm_client.send_request(1, "camera_home")
            #arm_client.get_logger().info(f'Response: {response.response}')    


            time.sleep(2)

            self.finish_move = True

            print("Impossible Mission Clear")

            self.armrun = False
            mission_status_msg = Bool() # std_msgs.msg.Bool 객체를 생성
            mission_status_msg.data = False
            self.mission_publisher.publish(mission_status_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()

    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()

    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500


    # Initial Position
    #ros2 topic list => ros2 topic echo /joint_states --once
    #point.positions =  [0.0, 0.5236, -0.7156, 0.8203]  # (단위: 라디안) <= 0, -44, 9, 72
    #point.positions = [0.0, 0.7317, -0.6918, 0.7701]    
    point.positions = [0.0, -0.488692, -0.453786, 2.007129] 

    trajectory_msg.points = [point]
    joint_pub.publish(trajectory_msg)

    try:
        #rclpy.spin_once(node)
        rclpy.spin(node) # 노드 실행
    except KeyboardInterrupt: # Ctrl+C 예외 처리
        pass
    finally:
        node.destroy_node() # 노드 자원 해제
        rclpy.shutdown() # ROS 시스템 종료

if __name__ == '__main__':
    main()