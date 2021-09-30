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
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
goal_x_list = [[24, 15,12,8],[24,13,7,8]]
goal_y_list = [[-8, -5, 0,8],[-8,-8,-2,8]]
class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('project/src',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_position = Pose()
        self.init_goal_x = -0
        self.init_goal_y = -15
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal2'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = 2
        self.last_goal_y = -15
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0
        self.mission=-1
    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal2":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass
    def lastPosition(self):
        return self.last_goal_x,self.last_goal_y
    def reset(self):
        self.index = 0
        self.mission = (self.mission + 1)%2
        self.last_goal_x = goal_x_list[self.mission][0]
        self.last_goal_y = goal_y_list[self.mission][0]
        return self.last_goal_x,self.last_goal_y
    def getPosition(self, position_check=False, delete=False,retry = False):
        if delete:
            self.deleteModel()
        #self.stage != 4
        if 0:
            while position_check:
                #goal_x = random.randrange(-12, 13) / 10.0
                #goal_y = random.randrange(-12, 13) / 10.0
                goal_x = random.randrange(-4, 9)
                goal_y = random.randrange(-9, 9) 
                '''
                if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False
                '''
                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        else:
            while position_check:
                if not retry:
                    self.index = (self.index + 1) % len(goal_x_list[self.mission])
                    self.goal_position.position.x = goal_x_list[self.mission][self.index]
                    self.goal_position.position.y = goal_y_list[self.mission][self.index]

                position_check = False
                

                position_check = False
                
        time.sleep(0.5)
        self.respawnModel()
        
        return self.goal_position.position.x, self.goal_position.position.y
