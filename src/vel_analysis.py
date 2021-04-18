#!/usr/bin/env python

import rospy
import time
import math
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates

class cal_vel:
    def __init__(self):
        self.predicted_vel = 0 # Velocity from simTracker
        self.model_pos_x = 0 # Position x from gazebo/model_states 
        self.model_pos_x_list = [] # list of Position x 
        self.model_vel = 0 # Velocity Calculated by using model_pos_x 
        self.length_before = 0   
        self.rmse_list = []
        self.rmse = 0
        self.time = 0 
        self.dt = 0

    def ModelCallback(self,input): # Callback ModelStates -> Obstacle Position
        self.model_pos_x = input.pose[2].position.x
        self.model_pos_x_list.append(self.model_pos_x)


    def cal_model_vel(self): # Calculate Model Velocity
        time_end = time.time()
        self.dt = time_end - self.time
        length = len(self.model_pos_x_list)-1
        self.model_vel = (self.model_pos_x_list[length] - self.model_pos_x_list[0])/ self.dt
        self.model_pos_x_list = []
        self.time = time_end


    def PredictCallback(self,input): # Callback Predicted Velocity from simTracker
        self.predicted_vel = float(input.markers[0].text)
        self.cal_model_vel()
        print("Tracked")
        print("pre_vel: ", self.predicted_vel)
        print("model_vel: ", self.model_vel)
        print("Error: ", abs(self.predicted_vel - abs(self.model_vel))*100/abs(self.model_vel))
        self.rmse_list.append((self.predicted_vel - abs(self.model_vel))**2)
        self.rmse = math.sqrt(sum(self.rmse_list)/len(self.rmse_list))
        print("rmse: ",self.rmse)


    def velocityListener(self):
        rospy.init_node('cal_vel',anonymous=True)
        self.time = time.time()
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.ModelCallback,queue_size=1)
        rospy.Subscriber("/tracker_viz", MarkerArray, self.PredictCallback,queue_size=1)
        rospy.spin()


if __name__ == "__main__":
    cal_vel = cal_vel()
    cal_vel.velocityListener()