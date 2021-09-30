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

import os
from random import randint, randrange
import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose,PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
from nav_msgs.msg import Path


from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
world = False

start = PoseStamped()
start.header.frame_id = "map"
start.pose.orientation.x = 0
start.pose.orientation.y = 0
start.pose.orientation.z = 0
start.pose.orientation.w = 1
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.z = 0
goal.pose.orientation.x = 10
goal.pose.orientation.y = 10
goal.pose.orientation.z = 0
goal.pose.orientation.w = 1

state_msg = ModelState()
state_msg.model_name = 'turtlebot3_burger'
state_msg.pose.position.x = 0
state_msg.pose.position.y = 0
state_msg.pose.position.z = 0
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = 0
state_msg.pose.orientation.w = 0

task_infx = [7,-9,7,-5,-9,1,13,-8,-5,9,7,14,-9,-9,6,-9,1,2,1,0]
task_infy = [-10,0,11,11,-4,9,14,11,4,1,11,-10,0,-10,-7,10,11,0,11,14]
from respawnGoal import Respawn
from respawnGoal_way import Respawn

from nav_msgs.srv import GetPlan
import time
import copy
target_not_movable = False
class Env():
    def __init__(self, action_dim=2,model = 1):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.yaw = 0
        self.waypoints = []
        self.waypoints_state = [0,0,0,0,0,0]
        self.position = Pose()

        self.task_index = 0
        if model == 1:
            self.model = 'turtlebot1'
            state_msg.model_name = 'turtlebot3_burger'
            self.pub_perform = rospy.Publisher('perform/agent_1', Float32, queue_size=5)
            #from hos_multi_goal_1 import Respawn
            from test_goal import Respawn
        else :
            self.model = 'turtlebot2'
            state_msg.model_name = 'turtlebot3_burger_2' 
            self.pub_perform = rospy.Publisher('perform/agent_2', Float32, queue_size=5)
            from hos_multi_goal_2 import Respawn

        self.pub_cmd_vel = rospy.Publisher(self.model+'/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber(self.model+'/odom', Odometry, self.getOdometry)
        self.pub_goal = rospy.ServiceProxy(self.model+'/move_base/make_plan',GetPlan)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def calculate_waypoint(self):
        start.pose.position.x = self.position.x
        start.pose.position.y = self.position.y
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        self.pub_goal(start,goal,0)
        rospy.loginfo("Calculate_waypoint")
        #rospy.sleep(1)
        
    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance
    
    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        #print 'yaw', yaw
        #print 'gA', goal_angle

        heading = goal_angle - yaw
        #print 'heading', heading
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)
        #self.update_waypoint()

    def getState_Reward(self,scan,action,past_action):
        
        scan_range = []
        heading = self.heading
        min_range = 0.136
        done = False

        waypoint_reward = 0

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf') or scan.ranges[i] == float('inf'):
                scan_range.append(10)
            elif np.isnan(scan.ranges[i]) or scan.ranges[i] == float('nan'):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        



        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        # current_distance = self.getGoalDistace()
        if current_distance < 0.3:
            self.get_goalbox = True
            
        linear_punish_reward = 0
        if action[0] < 0.2 :
            linear_punish_reward = -1.5

        angular_punish_reward = 0
        if abs(action[1]) > 1 :
            angular_punish_reward = -1
        '''
        if abs(action[0]) > 0.2: ###
            angular_punish_reward = angular_punish_reward - abs(heading)  ####
        '''
        distance_rate = (self.past_distance - current_distance) 
        
        if distance_rate > 0:
            distance_reward = 40.* distance_rate 
        else :
            distance_reward = 40.* distance_rate 

        self.past_distance = current_distance
        collision_reward = 0
        goal_reward = 0
 
        if min_range > min(scan_range) > 0: 
            done = True
            collision_reward = -50.
            rospy.loginfo("Collision!!")
            rospy.loginfo(scan_range)
            self.pub_cmd_vel.publish(Twist())
            self.pub_perform.publish(0)
        elif 0.18 > min(scan_range) >= min_range:
            collision_reward = -5.
        '''
        if current_distance > 8: 
            done = True 
            collision_reward = -50.
            rospy.loginfo("far away!!")
            rospy.loginfo(current_distance)
            self.pub_cmd_vel.publish(Twist())
        '''
        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            # reward = 500.
            goal_reward = 50.
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False
        reward = distance_reward + collision_reward + goal_reward + linear_punish_reward + angular_punish_reward
        if self.goal_x == 100 and self.goal_y == 100 :
            done = True 
            self.pub_perform.publish(1)
            self.respawn_goal.enable_update()
            #self.reset_path()
            self.goal_x = 101

        #rospy.loginfo(self.model + ':' + str(self.goal_x) + ','+ str(self.goal_y))
        #for pa in past_action :
        #    scan_range.append(pa)

        if current_distance > 5: 
            current_distance = 5 #
        return scan_range + [heading, current_distance], reward ,done
    def step(self, action, past_action):

        linear_vel = action[0]
        ang_vel = action[1]
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
   
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(self.model+'/scan', LaserScan, timeout=5)
            except:
                pass

        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
            #resp_pause = pause.call()
        #    self.pause_proxy()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/pause_physics service call failed")

        state,reward,done = self.getState_Reward(data,action,past_action)
        #reward, done = self.setReward(state, done)

        return np.asarray(state), reward, done
    def reset_path(self):
        if randint(0,1):
            state_msg.pose.position.x = randrange(10,13)
            state_msg.pose.position.y = randrange(0,10)
        else :
            state_msg.pose.position.x = randrange(-2,2)
            state_msg.pose.position.y = randrange(-2,2)

        state_msg.pose.position.x = task_infx[2*(self.task_index/10)]
        state_msg.pose.position.y = task_infy[2*(self.task_index/10)]
        start.pose.position.x = state_msg.pose.position.x
        start.pose.position.y = state_msg.pose.position.y
        goal.pose.position.x = task_infx[2*(self.task_index/10)+1]
        goal.pose.position.y = task_infy[2*(self.task_index/10)+1]
        self.task_index += 1
        self.pub_goal(start,goal,0)
        time.sleep(3)
        rospy.loginfo("Calculate_waypoint")
    def reset(self):
        #print('aqui2_____________---')
        self.respawn_goal.reset()
        if 1:
            if self.initGoal:
                '''
                rospy.wait_for_service('gazebo/reset_simulation')
                try:
                    self.reset_proxy()
                except (rospy.ServiceException) as e:
                    print("gazebo/reset_simulation service call failed")
                '''
                self.reset_path()
            #state_msg.pose.position.x,state_msg.pose.position.y = self.respawn_goal.lastPosition()
        self.reset_path()

    
        time.sleep(3)
        #state_msg.pose.position.x, state_msg.pose.position.y = self.respawn_goal.getPosition()
        #print(self.respawn_goal.getPosition())

        print(self.respawn_goal.lastPosition())
        state_msg.pose.position.x, state_msg.pose.position.y = self.respawn_goal.lastPosition()

        print("Respawn:"+ str(state_msg.pose.position.x)+','+str(state_msg.pose.position.y))
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except (rospy.ServiceException) as e:
            print("gazebo/set_model_state service call failed")

        self.unpause_proxy()

        #time.sleep(3)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(self.model+'/scan', LaserScan, timeout=5)
            except:
                pass

        
        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True)
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(delete=True) 
        self.goal_distance = self.getGoalDistace()
        #self.calculate_waypoint()
        state,c_,_= self.getState_Reward(data,[0]*self.action_dim,[0]*self.action_dim)
        #time.sleep(10)
        return np.asarray(state)
