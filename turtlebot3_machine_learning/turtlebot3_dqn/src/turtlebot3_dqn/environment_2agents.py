#!/usr/bin/env python
#################################################################################
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
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

import time

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading1 = 0
        self.heading2 = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position1 = Pose()
        self.position2 = Pose()
        self.pub_cmd_vel1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=5)
        self.sub_odom1 = rospy.Subscriber('tb3_0/odom', Odometry, self.getOdometry)
        self.pub_cmd_vel2 = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=5)
        self.sub_odom2 = rospy.Subscriber('tb3_1/odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.cmd_vel1 = 0.15
        self.cmd_vel2 = 0.15
        self.ang_vel1 = 0
        self.ang_vel2 = 0
        self.goal_y1 = 0.15
        self.goal_y2 = 0.15

    def getGoalDistace(self, scan_topic):
        if scan_topic == "tb3_0/scan":
            goal_distance = round(math.hypot(self.goal_x - self.position1.x, self.goal_y - self.position1.y), 2)
        elif scan_topic == "tb3_1/scan":
            goal_distance = round(math.hypot(self.goal_x - self.position2.x, self.goal_y - self.position2.y), 2)
        else:
            print("$%#$%GJDKAJFGLK")

        return goal_distance

    def getOdometry(self, odom):
        if odom.header.frame_id == "tb3_0/odom":
            self.position1 = odom.pose.pose.position
        elif odom.header.frame_id == "tb3_1/odom":
            self.position2 = odom.pose.pose.position
        else:
            print("rotlqkf")

        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        if odom.header.frame_id == "tb3_0/odom":
            goal_angle = math.atan2(self.goal_y - self.position1.y, self.goal_x - self.position1.x)
        elif odom.header.frame_id == "tb3_1/odom":
            goal_angle = math.atan2(self.goal_y - self.position2.y, self.goal_x - self.position2.x)
        else:
            print("##########################")

        heading = goal_angle - yaw
        # print(heading, goal_angle, yaw)

        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi
        
        if odom.header.frame_id == "tb3_0/odom":
            self.heading1 = round(heading, 2)
        elif odom.header.frame_id == "tb3_1/odom":
            self.heading2 = round(heading, 2)
        else:
            print("tlqkflqjflkjdlkfjqlwdjkf")

    def getState(self, scan, scan_topic):
        scan_range = []
        if scan_topic == "tb3_0/scan":
            heading = self.heading1
        elif scan_topic == "tb3_1/scan":
            heading = self.heading2
        else:
            print("@@@@@@@@@@@@@")

        min_range = 0.13 # 0.13
        done = False

        # print(scan_topic, scan.ranges)
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        distance_between1 = math.sqrt((self.position1.x - self.position2.x) ** 2 + (self.position1.y - self.position2.y) ** 2)
        distance_between = distance_between1
        # print("distance: ", distance_between)

        # print(self.position1.y, self.position2.y, self.position3.y)
        # print("1")
        # print(self.position1.x - self.position2.x, self.position1.y - self.position2.y)
        # print(self.position1.x - self.position3.x, self.position1.y - self.position3.y)
        # print("2")
        # print(self.position2.x - self.position1.x, self.position2.y - self.position1.y)
        # print(self.position2.x - self.position3.x, self.position2.y - self.position3.y)
        # print("3")
        # print(self.position3.x - self.position1.x, self.position3.y - self.position1.y)
        # print(self.position3.x - self.position2.x, self.position3.y - self.position2.y)
        state_rl = []
        if scan_topic == "tb3_0/scan":
            x12 = self.position1.x - self.position2.x
            y12 = self.position1.y - self.position2.y
            h12 = self.heading1 - self.heading2
            # state_rl = [x12, y12, h12]
            px = self.position1.x
            py = self.position1.y
            ph = self.heading1
            state_rl = [py, ph, x12]
        elif scan_topic == "tb3_1/scan":
            x21 = self.position2.x - self.position1.x
            y21 = self.position2.y - self.position1.y
            h21 = self.heading2 - self.heading1
            # state_rl = [x21, y21, h21]
            px = self.position2.x
            py = self.position2.y
            ph = self.heading2
            state_rl = [py, ph, x21]
        else:
            print("&&&&&&&&&&&&&&&&")

        exit1 = self.position1.y<0.05 or self.position1.y>0.25
        exit2 = self.position2.y<0.05 or self.position2.y>0.25
        exit_ = exit1 or exit2

        if (min_range > min(scan_range) > 0) or distance_between < 0.21 or exit_:
            done = True
        # print("scan_range: ", min(scan_range))
        if scan_topic == "tb3_0/scan":
            current_distance = round(math.hypot(self.goal_x - self.position1.x, self.goal_y - self.position1.y),2)
        elif scan_topic == "tb3_1/scan":
            current_distance = round(math.hypot(self.goal_x - self.position2.x, self.goal_y - self.position2.y),2)
        else:
            print("&&&&&&&&&&&&&&&&")

        if current_distance < 0.2:
            self.get_goalbox = True

        # return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done
        return [0] + state_rl, done

    def setReward(self, state, done, action, scan_topic):
        # yaw_reward = []
        # obstacle_min_range = state[-2]
        # current_distance = state[-3] # -1
        # heading = state[-4] # -2

        # for i in range(5):
        #     angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
        #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #     # print(tr)
        #     yaw_reward.append(tr)

        # # print(scan_topic)

        # if scan_topic == "tb3_0/scan":
        #     # print('1')
        #     distance_rate = 2 ** (current_distance / self.goal_distance1)
        # elif scan_topic == "tb3_1/scan":
        #     # print('2')
        #     distance_rate = 2 ** (current_distance / self.goal_distance2)
        # else:
        #     print("*********************")

        # if obstacle_min_range < 0.5:
        #     ob_reward = -5
        # else:
        #     ob_reward = 0

        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward
        # print(reward)
        # reward = 10

        if scan_topic == "tb3_0/scan":
            self.reward1 = self.position1.x
            reward = self.reward1 * 10 - abs(self.position1.y - 0.15) * 100
        elif scan_topic == "tb3_1/scan":
            self.reward2 = self.position2.x
            reward = self.reward2 * 10 - abs(self.position2.y - 0.15) * 100
        else:
            print("&&&&&&&&&&&&&&&&")

        # print(self.position1.y - 0.15)

        # reward += 10

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vel1.publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vel2.publish(Twist())
            else:
                print("77777777777777777777")

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200

            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vel1.publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vel2.publish(Twist())
            else:
                print("888888888888")

            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

            if scan_topic == "tb3_0/scan":
                self.goal_distance1 = self.getGoalDistace(scan_topic)
            elif scan_topic == "tb3_1/scan":
                self.goal_distance2 = self.getGoalDistace(scan_topic)
            else:
                print("(((((((((((((((((((((((")

            self.get_goalbox = False

        return reward

    def pid(self, goal_y, current_y):
        error = goal_y - current_y
        # print(error, goal_y, current_y)
        p = 0.5
        i = 0.1
        d = 0.01
        return error*p + (error/0.1)*d

    def set_vel_cmd(self, vel_cmd, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.cmd_vel1 = vel_cmd
        elif scan_topic == "tb3_1/scan":
            self.cmd_vel2 = vel_cmd
        else:
            print("(((((((((((((((((((((((")

    def set_ang_vel(self, ang_vel, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.ang_vel1 = ang_vel
        elif scan_topic == "tb3_1/scan":
            self.ang_vel2 = ang_vel
        else:
            print("(((((((((((((((((((((((")

    def set_goal_y(self, goal_y, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.goal_y1 = goal_y
        elif scan_topic == "tb3_1/scan":
            self.goal_y2 = goal_y
        else:
            print("(((((((((((((((((((((((")

    def straight(self, scan_topic):
        self.set_vel_cmd(0.1, scan_topic)
        self.set_ang_vel(0.0, scan_topic)
        # self.set_goal_y(0.15, scan_topic)

    def accelerate(self, scan_topic):
        self.set_vel_cmd(0.15, scan_topic)
        self.set_ang_vel(0.0, scan_topic)
        # self.set_goal_y(0.15, scan_topic)

    def decelerate(self, scan_topic):
        self.set_vel_cmd(0.05, scan_topic)
        self.set_ang_vel(0.0, scan_topic)
        # self.set_goal_y(0.15, scan_topic)

    def stop(self, scan_topic):
        # time.sleep(1)
        self.set_vel_cmd(0.0, scan_topic)
        self.set_ang_vel(0.0, scan_topic)
        
        # self.set_goal_y(0.15, scan_topic)

    def turn_left(self, scan_topic):
        # self.set_vel_cmd(0.15, scan_topic)
        self.set_goal_y(0.15, scan_topic)
        self.set_ang_vel(-0.5, scan_topic)
        # print(self.ang_vel2)

    def turn_right(self, scan_topic):
        # self.set_vel_cmd(0.15, scan_topic)
        self.set_goal_y(-0.15, scan_topic)
        self.set_ang_vel(0.5, scan_topic)

    def step(self, action, scan_topic):
        max_angular_vel = 1.0 # 1.5
        # ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        actions = [self.straight, self.accelerate, self.decelerate, self.stop, self.turn_left, self.turn_right]
        actions[action](scan_topic)
        # print(action)

        # if scan_topic == "tb3_0/scan":
        #     actions[action](scan_topic)
        # elif scan_topic == "tb3_1/scan":
        #     actions[2](scan_topic)
        # else:
        #     print("tlqkf")

        if scan_topic == "tb3_0/scan":
            ang_vel = self.ang_vel1 + self.pid(self.goal_y1, self.position1.y)
            # print(self.pid(0.15, self.position1.y))
            # print("1", ang_vel)
        elif scan_topic == "tb3_1/scan":
            ang_vel = self.ang_vel2 + self.pid(self.goal_y2, self.position2.y)
            # print("2", ang_vel)
        else:
            print("tlqkf")

        # ang_vel = min(ang_vel, max_angular_vel)
        # self.set_ang_vel(scan_topic)

        # print(action)

        vel_cmd = Twist()
        # vel_cmd.linear.x = 0.15
        # vel_cmd.angular.z = ang_vel

        if scan_topic == "tb3_0/scan":
            vel_cmd.linear.x = self.cmd_vel1
            vel_cmd.angular.z = self.ang_vel1
        elif scan_topic == "tb3_1/scan":
            vel_cmd.linear.x = self.cmd_vel2
            vel_cmd.angular.z = self.ang_vel2
        else:
            print("tlqkf")
        # print(self.ang_vel1, self.ang_vel2)
        # print(self.goal_y1, self.goal_y2, self.goal_y3)

        if scan_topic == "tb3_0/scan":
            self.pub_cmd_vel1.publish(vel_cmd)
        elif scan_topic == "tb3_1/scan":
            self.pub_cmd_vel2.publish(vel_cmd)
        else:
            print("tlqkf")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(scan_topic, LaserScan, timeout=5)
            except:
                pass

        time.sleep(0.1)

        state, done = self.getState(data, scan_topic)
        reward = self.setReward(state, done, action, scan_topic)

        # print(self.position1.x, self.position1.y)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message("tb3_0/scan", LaserScan, timeout=5)
            except:
                pass
        data2 = None
        while data2 is None:
            try:
                data2 = rospy.wait_for_message("tb3_1/scan", LaserScan, timeout=5)
            except:
                pass
        data3 = None

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        # if scan_topic == "tb3_0/scan":
        self.goal_distance1 = self.getGoalDistace("tb3_0/scan")
        # elif scan_topic == "tb3_1/scan":
        self.goal_distance2 = self.getGoalDistace("tb3_1/scan")
        # else:
            # print("))))))))))))")

        state1, done1 = self.getState(data1, "tb3_0/scan")
        state2, done2 = self.getState(data2, "tb3_1/scan")

        return np.asarray(state1), np.asarray(state2)