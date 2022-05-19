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
    def __init__(self, agents, action_size):
        self.agents = agents
        self.num_agents = len(agents)
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

        self.dones = [False for _ in range(self.num_agents)]
        self.goals = [False for _ in range(self.num_agents)]

        self.goal = False
        self.goal_num=0
        self.sequence=0
        

        # self.initGoal = True
        # self.get_goalbox = False
        # self.respawn_goal = Respawn()

    # def getGoalDistance(self):
    #     goal_distances = []
    #     for idx in range(self.num_agents):
    #         goal_distances.append(round(math.hypot(self.goal_xs[idx] - self.positions[idx].x, 0), 2))
    #     return goal_distances

    def getOdometry(self, odom):
        idx = int(odom.header.frame_id[4])

        self.positions[idx] = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaws[idx] = euler_from_quaternion(orientation_list)

        # goal_angle = math.atan2(self.goal_ys[idx] - self.positions[idx].y, self.goal_xs[idx] - self.positions[idx].x)
        # heading = goal_angle - self.yaws[idx]
        # if heading > pi:
        #     heading -= 2 * pi
        # elif heading < -pi:
        #     heading += 2 * pi
        # self.headings[idx] = round(heading, 2)

    def getState(self):
        states = []
        dones = []
        pxes = []

        distance_betweens = []
        exits = []

        for idx in range(self.num_agents):
            min_range = 0.13 
            done = False
            self.dones[idx] = False
            self.goals[idx] = False
            
            state = []
            state.append(round(math.hypot(self.goal_xs[idx] - self.positions[idx].x, 0),2))
            pxes.append(self.positions[idx].x)

            state.append(round(self.positions[idx].y, 2))
            state.append(round(self.headings[idx], 2))
            for j in range(self.num_agents):
                if idx == j:
                    pass
                else:
                    state.append(round(self.positions[idx].x - self.positions[j].x, 2))
                    state.append(round(self.positions[idx].y - self.positions[j].y, 2))
                    state.append(round(self.headings[idx] - self.headings[j], 2))
                    distance_betweens.append(math.sqrt((self.positions[idx].x - self.positions[j].x) ** 2 + (self.positions[idx].y - self.positions[j].y) ** 2))
            states.append(state)

            exit = self.positions[idx].y < -0.25 or self.positions[idx].y > 0.25
            goal_done = self.positions[0].x > 4.0 # 22-01-05 
            distance_between = min(distance_betweens)
            if distance_between < 0.21 or exit or goal_done:
                done = True
                # self.done = True
            else:
                done = False
            dones.append(done)
            
            # if current_distance < 0.2 or goal_done:
            if goal_done:
                self.get_goalbox = True
                self.goals[idx] = True

        return states, dones, pxes

    def setReward(self, states, dones, actions, agent_topics):
        yaw_reward = []
        # obstacle_min_range = state[-2]
        current_distance = states[-1] # -1 # -3
        heading = states[-2] # -2 # -4

        rewards = []
        for idx in range(self.num_agents):
            if self.positions[0].x>self.positions[2].x:
                self.sequence+=1
                
            if self.sequence is 1:
                reward=200
            else:
                reward=-1
                
            # reward = self.positions[0].x-self.positions[2].x# - abs(reward_y) # 22-01-05
            
            # if self.positions[0].x<self.positions[2].x:
            #     reward=-1
            # else:
            #     reward = self.positions[0].x
            
            # reward = -1
            
            # reward = self.positions[0].x
                
            # print('[0].x: %f[1].x: %f[2].x: %f'%(self.positions[0].x,self.positions[1].x,self.positions[2].x))
            
            # if done or self.done:
            if dones[0]:
                rospy.loginfo("Collision!!")
                reward = -200 # -200
                self.pub_cmd_vels[idx].publish(Twist())
                print("self.goal_num:")
                print(self.goal_num)

            # if self.get_goalbox or self.goal:
            if self.goals[0]:
                rospy.loginfo("Goal!!")
                reward = 200
                self.pub_cmd_vels[idx].publish(Twist())
                self.goal_num=self.goal_num+1

                # self.goal_distances = self.getGoalDistance()
                self.get_goalbox = False
                

            rewards.append(reward)
        # print('rewards:%f'%rewards[0])
        return rewards

    def setAction(self, actions, agent_topics):
        action_list = [self.turn_left_f, self.turn_left_h, self.turn_left_m, self.turn_left_l, self.turn_right_f, self.turn_right_h, self.turn_right_m, self.turn_right_l, self.stop]
        
        pid_list = [self.pid, self.pid_2, self.pid_3]
        input_list = [self.goal_ys[0], -0.15, 0.15]
        vel_cmds = [Twist() for _ in range(self.num_agents)]

        for idx in range(self.num_agents):
            action_list[actions[idx]](idx)

            vel_cmds[idx].linear.x = self.cmd_vels[idx]
            vel_cmds[idx].angular.z = pid_list[idx](input_list[idx]) # 
            self.pub_cmd_vels[idx].publish(vel_cmds[idx])

            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message(agent_topics[idx] + "/scan", LaserScan, timeout=5)
                except:
                    pass

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

    def set_vel_cmd(self, vel_cmd, idx):
        self.cmd_vels[idx] = vel_cmd

    def set_goal_y(self, goal_y, idx):
        self.goal_ys[idx] = goal_y

    def turn_left_f(self, idx):
        self.set_vel_cmd(0.20, idx)
        self.set_goal_y(0.15, idx)
        return "0"

    def turn_left_h(self, idx):
        self.set_vel_cmd(0.15, idx)
        self.set_goal_y(0.15, idx)
        return "1"

    def turn_left_m(self, idx):
        self.set_vel_cmd(0.1, idx)
        self.set_goal_y(0.15, idx)
        return "2"

    def turn_left_l(self, idx):
        self.set_vel_cmd(0.05, idx)
        self.set_goal_y(0.15, idx)
        return "3"

    def turn_right_f(self, idx):
        self.set_vel_cmd(0.20, idx)
        self.set_goal_y(-0.15, idx)
        return "4"

    def turn_right_h(self, idx):
        self.set_vel_cmd(0.15, idx)
        self.set_goal_y(-0.15, idx)
        return "5"

    def turn_right_m(self, idx):
        self.set_vel_cmd(0.1, idx)
        self.set_goal_y(-0.15, idx)
        return "6"

    def turn_right_l(self, idx):
        self.set_vel_cmd(0.05, idx)
        self.set_goal_y(-0.15, idx)
        return "7"

    def stop(self, idx):
        self.set_vel_cmd(0.0, idx)
        return "8"

    def step(self, actions, agent_topics):
        self.setAction(actions, agent_topics)
        

        
        # next_states = [0 for _ in range(self.num_agents)]
        # rewards = [0 for _ in range(self.num_agents)]
        # dones = [0 for _ in range(self.num_agents)]
        # next_pxes = [0 for _ in range(self.num_agents)]

        next_states, dones, next_pxes = self.getState()
        rewards = self.setReward(next_states, dones, actions, agent_topics)

        return np.asarray(next_states), rewards, dones, next_pxes

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
        while data3 is None:
            try:
                data3 = rospy.wait_for_message("tb3_2/scan", LaserScan, timeout=5)
            except:
                pass
        
        # self.goal_distances = self.getGoalDistance()
        # data = [data1, data2, data3]
        states, dones, pxes = self.getState()

        return np.asarray(states), pxes