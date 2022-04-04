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
# Revised by hayalee #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from respawnGoal import Respawn

class Env():
    def __init__(self, num_agents, action_size):
        self.num_agents = num_agents
        self.agent_topics = ["tb3_{}".format(i) for i in range(self.num_agents)]
        self.action_size = action_size

        self.pub_cmd_vels = [rospy.Publisher(topic, Twist, queue_size=5) for topic in [self.agent_topics[i] + "/cmd_vel" for i in range(self.num_agents)]]
        self.sub_odoms = [rospy.Subscriber(topic, Odometry, self.getOdometry) for topic in [self.agent_topics[i] + "/odom" for i in range(self.num_agents)]]
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.goal_xs = [5.0 for _ in range(self.num_agents)]
        self.goal_ys = [0.15 for _ in range(self.num_agents)]
        self.goal_ys[1] = -0.15
        
        self.positions = [Pose() for _ in range(self.num_agents)]
        self.yaws = [0.0 for _ in range(self.num_agents)]
        self.headings = [0.0 for _ in range(self.num_agents)]
        
        self.cmd_vels = [0.15 for _ in range(self.num_agents)]
        self.ang_vels = [0.0 for _ in range(self.num_agents)]

        self.done = False
        self.goal = False

        # self.initGoal = True
        # self.get_goalbox = False
        # self.respawn_goal = Respawn()

    def getGoalDistance(self):
        goal_distances = []
        for idx in range(self.num_agents):
            goal_distances.append(round(math.hypot(self.goal_xs[idx] - self.positions[idx].x, 0), 2))
        return goal_distances

    def getOdometry(self, odom):
        if odom.header.frame_id == "tb3_0/odom":
            self.positions[0] = odom.pose.pose.position

            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, self.yaws[0] = euler_from_quaternion(orientation_list)

            goal_angle = math.atan2(self.goal_ys[0] - self.positions[0].y, self.goal_xs[0] - self.positions[0].x)
            heading = goal_angle - self.yaws[0]
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            self.headings[0] = round(heading, 2)
        elif odom.header.frame_id == "tb3_1/odom":
            self.positions[1] = odom.pose.pose.position

            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, self.yaws[1] = euler_from_quaternion(orientation_list)

            goal_angle = math.atan2(self.goal_ys[1] - self.positions[1].y, self.goal_xs[1] - self.positions[1].x)
            heading = goal_angle - self.yaws[1]
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            self.headings[1] = round(heading, 2)
        elif odom.header.frame_id == "tb3_2/odom":
            self.positions[2] = odom.pose.pose.position

            orientation = odom.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, self.yaws[2] = euler_from_quaternion(orientation_list)

            goal_angle = math.atan2(self.goal_ys[2] - self.positions[2].y, self.goal_xs[2] - self.positions[2].x)
            heading = goal_angle - self.yaws[2]
            if heading > pi:
                heading -= 2 * pi
            elif heading < -pi:
                heading += 2 * pi
            self.headings[2] = round(heading, 2)
        else:
            pass

    def getState(self, scan):
        for idx in range(self.num_agents):
            scan_range = []
            heading = self.headings[idx]
            min_range = 0.13 # 0.13
            done = False
            if scan_topic == "tb3_0/scan":
                self.done = False
                self.goal = False
            elif scan_topic == "tb3_1/scan":
                pass
            elif scan_topic == "tb3_2/scan":
                pass
            else:
                pass

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
            distance_between2 = math.sqrt((self.position1.x - self.position3.x) ** 2 + (self.position1.y - self.position3.y) ** 2)
            distance_between3 = math.sqrt((self.position3.x - self.position2.x) ** 2 + (self.position3.y - self.position2.y) ** 2)
            distance_between = min(distance_between1, distance_between2, distance_between3)
            
            state_rl = []
            if scan_topic == "tb3_0/scan":
                x12 = round(self.position1.x - self.position2.x, 2)
                if x12 > 1.0:
                    x12 = 1.0
                elif x12 < -1.0:
                    x12 = -1.0
                else:
                    pass
                y12 = round(self.position1.y - self.position2.y, 2)
                h12 = round(self.heading1 - self.heading2, 2)
                x13 = round(self.position1.x - self.position3.x, 2)
                if x13 > 1.0:
                    x13 = 1.0
                elif x13 < -1.0:
                    x13 = -1.0
                else:
                    pass
                y13 = round(self.position1.y - self.position3.y, 2)
                h13 = round(self.heading1 - self.heading3, 2)
                px = self.position1.x
                py = round(self.position1.y, 2)
                ph = round(self.heading1, 2)
                state_rl = [py, ph, x12, y12, h12, x13, y13, h13]
                # print(state_rl)
            elif scan_topic == "tb3_1/scan":
                x21 = round(self.position2.x - self.position1.x, 2)
                y21 = round(self.position2.y - self.position1.y, 2)
                h21 = round(self.heading2 - self.heading1, 2)
                x23 = round(self.position2.x - self.position3.x, 2)
                y23 = round(self.position2.y - self.position3.y, 2)
                h23 = round(self.heading2 - self.heading3, 2)
                px = self.position2.x
                py = round(self.position2.y, 2)
                ph = round(self.heading2, 2)
                state_rl = [py, ph, x21, y21, h21, x23, y23, h23]
            elif scan_topic == "tb3_2/scan":
                x31 = round(self.position3.x - self.position1.x, 2)
                y31 = round(self.position3.y - self.position1.y, 2)
                h31 = round(self.heading3 - self.heading1, 2)
                x32 = round(self.position3.x - self.position2.x, 2)
                y32 = round(self.position3.y - self.position2.y, 2)
                h32 = round(self.heading3 - self.heading2, 2)
                px = self.position3.x
                py = round(self.position3.y, 2)
                ph = round(self.heading3, 2)
                state_rl = [py, ph, x31, y31, h31, x32, y32, h32]
            else:
                pass

            exit1 = self.position1.y < -0.25 or self.position1.y > 0.25
            exit2 = self.position2.y < -0.25 or self.position2.y > 0.25
            exit3 = self.position3.y < -0.25 or self.position3.y > 0.25
            exit_ = exit1 or exit2 or exit3

            goal_done = self.position1.x > 5.0 # 22-01-05 

            if (min_range > min(scan_range) > 0) or distance_between < 0.21 or exit_ or goal_done:
                # done = True
                if scan_topic == "tb3_0/scan":
                    done = True
                    self.done = True
                elif scan_topic == "tb3_1/scan":
                    pass
                elif scan_topic == "tb3_2/scan":
                    pass
                else:
                    pass
                
            if scan_topic == "tb3_0/scan":
                # current_distance = round(math.hypot(self.goal_x - self.position1.x, self.goal_y - self.position1.y),2)
                current_distance = round(math.hypot(self.goal_x - self.position1.x, 0),2)
                # print(current_distance)
            elif scan_topic == "tb3_1/scan":
                # current_distance = round(math.hypot(self.goal_x - self.position2.x, self.goal_y - self.position2.y),2)
                current_distance = round(math.hypot(self.goal_x - self.position2.x, 0),2)
            elif scan_topic == "tb3_2/scan":
                # current_distance = round(math.hypot(self.goal_x - self.position3.x, self.goal_y - self.position3.y),2)
                current_distance = round(math.hypot(self.goal_x - self.position3.x, 0),2)
            else:
                pass

            # if current_distance < 0.2 or goal_done:
            if goal_done:
                if scan_topic == "tb3_0/scan":
                    self.get_goalbox = True
                    self.goal = True
                elif scan_topic == "tb3_1/scan":
                    pass
                elif scan_topic == "tb3_2/scan":
                    pass
                else:
                    pass
        return [current_distance] + state_rl , done, px # [0, 0] + [round(heading, 2), round(current_distance, 2)]

    def setReward(self, state, done, action, scan_topic):
        yaw_reward = []
        # obstacle_min_range = state[-2]
        current_distance = state[-1] # -1 # -3
        heading = state[-2] # -2 # -4

        reward = -1 # - abs(reward_y) # 22-01-05

        if done or self.done:
            rospy.loginfo("Collision!!")
            reward = -1 # -200
            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vels[0].publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vels[1].publish(Twist())
            elif scan_topic == "tb3_2/scan":
                self.pub_cmd_vels[2].publish(Twist())
            else:
                pass

        # if self.get_goalbox or self.goal:
        if self.goal:
            rospy.loginfo("Goal!!")
            reward = 100
            if scan_topic == "tb3_0/scan":
                self.pub_cmd_vels[0].publish(Twist())
            elif scan_topic == "tb3_1/scan":
                self.pub_cmd_vels[1].publish(Twist())
            elif scan_topic == "tb3_2/scan":
                self.pub_cmd_vels[2].publish(Twist())
            else:
                pass

            # self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True) # 22-01-07

            # if scan_topic == "tb3_0/scan":
            #     self.goal_distance1 = self.getGoalDistance(scan_topic)
            # elif scan_topic == "tb3_1/scan":
            #     self.goal_distance2 = self.getGoalDistance(scan_topic)
            # elif scan_topic == "tb3_2/scan":
            #     self.goal_distance3 = self.getGoalDistance(scan_topic)
            # else:
            #     pass
            self.goal_distances = self.getGoalDistance()

            self.get_goalbox = False
        return reward

    def pid(self, goal_y):
        p = 1.0
        i = 0.0
        d = 0.0
        y_g_pos = goal_y 
        y_c_pos = self.positions[0].y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaws[0] 
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error 
        result = p * error

        if result < -0.001:
            if result < -0.2:
                if self.yaws[0] < -pi/2:
                    result = 0.0
                else:
                    result = -0.6
        if result > 0.001:
            if result > 0.2:
                if self.yaws[0] > pi/2:
                    result = 0.0
                else:
                    result = 0.6
        return result 

    def pid_2(self, goal_y): 
        p = 1.0
        y_g_pos = goal_y 
        y_c_pos = self.positions[1].y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaws[1] 
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error 
        result = p * error
        return result 

    def pid_3(self, goal_y): 
        p = 1.0
        y_g_pos = goal_y 
        y_c_pos = self.positions[2].y
        p_error = y_g_pos - y_c_pos
        y_error = 0 - self.yaws[2] 
        ang_error = math.atan2(p_error,0.3)
        error = 1.5 * y_error + ang_error 
        result = p * error
        return result 

    def set_vel_cmd(self, vel_cmd, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.cmd_vels[0] = vel_cmd
        elif scan_topic == "tb3_1/scan":
            self.cmd_vels[1] = vel_cmd
        elif scan_topic == "tb3_2/scan":
            self.cmd_vels[2] = vel_cmd
        else:
            pass

    def set_ang_vel(self, ang_vel, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.ang_vels[0] = ang_vel
        elif scan_topic == "tb3_1/scan":
            self.ang_vels[1] = ang_vel
        elif scan_topic == "tb3_2/scan":
            self.ang_vels[2] = ang_vel
        else:
            pass

    def set_goal_y(self, goal_y, scan_topic):
        if scan_topic == "tb3_0/scan":
            self.goal_ys[0] = goal_y
        elif scan_topic == "tb3_1/scan":
            self.goal_ys[1] = goal_y
        elif scan_topic == "tb3_2/scan":
            self.goal_ys[2] = goal_y
        else:
            pass

    def turn_left_f(self, scan_topic):
        self.set_vel_cmd(0.20, scan_topic)
        self.set_goal_y(0.15, scan_topic)
        return "0"

    def turn_left_h(self, scan_topic):
        self.set_vel_cmd(0.15, scan_topic)
        self.set_goal_y(0.15, scan_topic)
        return "1"

    def turn_left_m(self, scan_topic):
        self.set_vel_cmd(0.1, scan_topic)
        self.set_goal_y(0.15, scan_topic)
        return "2"

    def turn_left_l(self, scan_topic):
        self.set_vel_cmd(0.05, scan_topic)
        self.set_goal_y(0.15, scan_topic)
        return "3"

    def turn_right_f(self, scan_topic):
        self.set_vel_cmd(0.20, scan_topic)
        self.set_goal_y(-0.15, scan_topic)
        return "4"

    def turn_right_h(self, scan_topic):
        self.set_vel_cmd(0.15, scan_topic)
        self.set_goal_y(-0.15, scan_topic)
        return "5"

    def turn_right_m(self, scan_topic):
        self.set_vel_cmd(0.1, scan_topic)
        self.set_goal_y(-0.15, scan_topic)
        return "6"

    def turn_right_l(self, scan_topic):
        self.set_vel_cmd(0.05, scan_topic)
        self.set_goal_y(-0.15, scan_topic)
        return "7"

    def stop(self, scan_topic):
        self.set_vel_cmd(0.0, scan_topic)
        return "8"

    def step(self, actions, agent_topics):
        action_list = [self.turn_left_f, self.turn_left_h, self.turn_left_m, self.turn_left_l, self.turn_right_f, self.turn_right_h, self.turn_right_m, self.turn_right_l, self.stop]
        
        vel_cmds = [Twist() for _ in range(self.num_agents)]

        next_states = [0 for _ in range(self.num_agents)]
        rewards = [0 for _ in range(self.num_agents)]
        dones = [0 for _ in range(self.num_agents)]
        next_pxes = [0 for _ in range(self.num_agents)]

        for idx in range(self.num_agents):
            action_list[actions[idx]](agent_topics[idx])
            print(action_list[actions[idx]](agent_topics[idx]), actions[idx], agent_topics[idx])

            vel_cmds[idx].linear.x = self.cmd_vels[idx]
            # vel_cmds[idx].angular.z = self.pid(self.goal_ys[idx])


            if agent_topics == "tb3_0/scan":
            #     vel_cmd1.linear.x = self.cmd_vel1 
                vel_cmds[idx].angular.z = self.pid(self.goal_ys[idx]) # here
            elif agent_topics == "tb3_1/scan":
            #     vel_cmd2.linear.x = self.cmd_vel2
                vel_cmds[idx].angular.z = self.pid_2(-0.15) # self.ang_vel2
            elif agent_topics == "tb3_2/scan":
            #     vel_cmd3.linear.x = self.cmd_vel3
                vel_cmds[idx].angular.z = self.pid_3(0.15) # self.ang_vel3 
            else:
                pass
            print(idx, vel_cmds[idx].linear.x, vel_cmds[idx].angular.z)
            self.pub_cmd_vels[idx].publish(vel_cmds[idx])

            # if agent_topics == "tb3_0/scan":
            #     self.pub_cmd_vel1.publish(vel_cmd1)
            # elif agent_topics == "tb3_1/scan":
            #     self.pub_cmd_vel2.publish(vel_cmd2)
            # elif agent_topics == "tb3_2/scan":
            #     self.pub_cmd_vel3.publish(vel_cmd3)
            # else:
            #     pass

            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message(agent_topics[idx], LaserScan, timeout=5)
                except:
                    pass

            next_states[idx], dones[idx], next_pxes[idx] = self.getState(data, agent_topics[idx])
            rewards[idx] = self.setReward(next_states[idx], dones[idx], actions[idx], agent_topics[idx])

        # return np.asarray(state), reward, done, px
        return np.asarray(next_states), rewards, dones, next_pxes

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        # data = [None for _ in range(self.num_agents)]
        # while data is None:
        #     try:
        #         for idx in range(self.num_agents):
        #             data[idx] = rospy.wait_for_message(self.agent_topics[idx], LaserScan, timeout=5)
        #     except:
        #         pass
        
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
        while data3 is None:
            try:
                data3 = rospy.wait_for_message("tb3_2/scan", LaserScan, timeout=5)
            except:
                pass

        # if self.initGoal:
            # self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            # self.goal_x = 5.0
            # self.goal_y = 0.0
            # self.initGoal = False

        # self.goal_distance1 = self.getGoalDistance("tb3_0/scan")
        # self.goal_distance2 = self.getGoalDistance("tb3_1/scan")
        # self.goal_distance3 = self.getGoalDistance("tb3_2/scan")
        self.goal_distances = self.getGoalDistance()

        data = [data1, data2, data3]
        # state1, done1, px1 = self.getState(data1, "tb3_0/scan")
        # state2, done2, px2 = self.getState(data2, "tb3_1/scan")
        # state3, done3, px3 = self.getState(data3, "tb3_2/scan")
        states, dones, pxes = self.getState(data)

        # states = [state1, state2, state3]
        # pxes = [px1, px2, px3]

        return np.asarray(states), pxes