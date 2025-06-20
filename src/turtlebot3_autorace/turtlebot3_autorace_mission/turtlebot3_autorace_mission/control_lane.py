#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.sub_mission_state = self.create_subscription(
            Bool,
            '/mission_state',
            self.callback_mission_state,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )


        # PD control related variables
        self.last_error = 0
        self.filtered_error = 0.0
        self.MAX_VEL = 0.07

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()

        self.stop_state = False

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_follow_lane(self, desired_center):
        if self.stop_state == False:
            """
            Receive lane center data to generate lane following control commands.

            If avoidance mode is enabled, lane following control is ignored.
            """
            if self.avoid_active:
                return

            center = desired_center.data
            error = center - 320

            # Kp = 0.0025
            # Kd = 0.007

            # angular_z = Kp * error + Kd * (error - self.last_error)
            # self.last_error = error

            # twist = Twist()
            # # Linear velocity: adjust speed based on error (maximum 0.05 limit)
            # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
            # twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            # self.pub_cmd_vel.publish(twist)

            # 조정된 Kp, Kd는 그대로 유지
            # Kp = 0.0025
            # Kd = 0.007

            # error = center - 500
            # angular_z = Kp * error + Kd * (error - self.last_error)
            # self.last_error = error

            # twist = Twist()

            # # 새로 조정한 속도 공식: error가 작을수록 속도는 MAX_VEL에 가까워짐
            # speed_factor = max(1 - abs(error) / 500, 0)
            # twist.linear.x = min(self.MAX_VEL * (speed_factor ** 1.5), self.MAX_VEL)

            # # 회전은 기존대로 제한
            # twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)

            # self.pub_cmd_vel.publish(twist)

            # alpha = 0.3
            # self.filtered_error = alpha * error + (1 - alpha) * self.filtered_error
            # error = self.filtered_error

            # # 차선 유실 판단 및 회복 로직
            # if abs(error) > 400:  # 기준은 적절히 조정
            #     self.lost_lane_count += 1
            # else:
            #     self.lost_lane_count = 0

            # # 일정 시간 이상 유실되면 회전하여 탐색
            # if self.lost_lane_count > 5:
            #     self.get_logger().warn("Lane lost. Rotating to search...")
            #     twist = Twist()
            #     twist.linear.x = 0.0
            #     twist.angular.z = 0.4  # 회전 속도
            #     self.pub_cmd_vel.publish(twist)
            #     return

            # PD 제어
            Kp = 0.0027 # org = 0.0025
            Kd = 0.0063  # org = 0.007
            angular_z = Kp * error + Kd * (error - self.last_error)
            self.last_error = error

            # 속도 계산
            speed_factor = max(1 - abs(error) / 320, 0)
            twist = Twist()
            twist.linear.x = min(self.MAX_VEL * (speed_factor ** 1.8), self.MAX_VEL)

            # 회전 속도 제한
            twist.angular.z = -max(angular_z, -1.2) if angular_z < 0 else -min(angular_z, 1.2) # 0.75

            # 조향 강도에 따른 속도 감속
            angular_factor = 1.0 - min(abs(twist.angular.z) / 1.0, 1.0)
            twist.linear.x *= angular_factor

            self.pub_cmd_vel.publish(twist)
        else:
            twist = Twist()
            twist.linear.x =  0.0
            twist.angular.z =  0.0
            self.pub_cmd_vel.publish(twist)


    def callback_avoid_cmd(self, twist_msg):
        if self.stop_state == False:
            self.avoid_twist = twist_msg

            if self.avoid_active:
                self.pub_cmd_vel.publish(self.avoid_twist)
        else:
            twist = Twist()
            twist.linear.x =  0.0
            twist.angular.z =  0.0
            self.pub_cmd_vel.publish(twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def shut_down(self):

        twist = Twist()
        self.pub_cmd_vel.publish(twist)

    def callback_mission_state(self,bool_msg):
        if bool_msg.data:
            self.get_logger().info('stop_state  ============================= True')
            self.stop_state = True
        else:
            self.get_logger().info('stop_state  ============================= False')
            self.stop_state = False
        

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()