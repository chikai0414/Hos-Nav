import rospy
import random
import os 
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

import math
class car():
    def __init__(self):
        self.x = 0
        self.y = 0

class noise():
    def __init__(self):
        self.sub_ped = rospy.Subscriber('gazebo/model_states',ModelStates,self.checkModel)
        self.db_1 = 0
        self.db_2 = 0

        self.car1 = car()
        self.car1.x = 0
        self.car1.y = 0
        
        self.car2 = car()
        self.car2.x = 0
        self.car2.y = 0

        self.pub_noise = rospy.Publisher('turtlebot1/noise',Float32,queue_size=5)
        self.pub_noise2 = rospy.Publisher('turtlebot2/noise',Float32,queue_size=5)

        #self.pub_noise = rospy.Publisher('nosie',Float32,queue_size=5)

        

    def checkModel(self,model):
        for i in range(len(model.name)):
            if model.name[i] == "turtlebot3_burger":
                self.car1.x = model.pose[i].position.x
                self.car1.y = model.pose[i].position.y
            if model.name[i] == "turtlebot3_burger_2":
                self.car2.x = model.pose[i].position.x
                self.car2.y = model.pose[i].position.y
            if model.name[i].find("sick") !=-1 or model.name[i].find("nurse")!=-1 and model.name[i].find("collision_model") == -1:
                distance_1 = math.hypot(model.pose[i].position.x - self.car1.x, model.pose[i].position.y - self.car1.y)
                if distance_1 <= 1:
                    _db_1 = 40
                else:
                    _db_1 = 40 - 6 * math.log(distance_1,2)
                #print(model.namse[i] + ": "+str(_db))
                self.db_1 = 10* math.log10(pow(10,self.db_1/10)+pow(10,_db_1/10))

                distance_2 = math.hypot(model.pose[i].position.x - self.car2.x, model.pose[i].position.y - self.car2.y)
                if distance_2 <= 1:
                    _db_2 = 40
                else:
                    _db_2 = 40 - 6 * math.log(distance_2,2)
                #print(model.namse[i] + ": "+str(_db))
                self.db_2 = 10* math.log10(pow(10,self.db_2/10)+pow(10,_db_2/10))
        print(self.db_1)
        print(self.db_2)
        self.pub_noise.publish(self.db_1)
        self.pub_noise2.publish(self.db_2)

        self.db_1 = 0
        self.db_2 = 0

if __name__ == '__main__':
    rospy.init_node("noise")
    n = noise()
    rospy.spin()