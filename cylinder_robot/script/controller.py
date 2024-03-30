#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import numpy as np
import math



class PID_Controller:
 
    def __init__(self,kp,ki,kd,output_min,output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0 
        self.last_error = 0 
        self.error_sum = 0
        self.error_diff = 0
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = 30
        self.output = 0

    def constrain(self, output):
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error
        self.error_sum = max(min(self.error_sum, self.integral_limit), -self.integral_limit)
        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)
        return self.output

class controller(object):
    def __init__(self):
        self.pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
        rospy.Subscriber("/robot/esti_model_state",ModelState, self.callback)
        self.x, self.y, self.vx, self.vy,self.last_x, self.last_y = -2,-2,0,0,0,0
        self.time_now,self.last_time = 0,0
        self.twist = Twist()
        self.threshold_distance = 0.05

        self.control_x = PID_Controller(1.3, 0, 0.4, -0.6, 0.6) 
        self.control_vx = PID_Controller(10,0, 0.01, -18, 18)
        self.control_y = PID_Controller(1.3, 0, 0.4, -0.6, 0.6) 
        self.control_vy = PID_Controller(10,0, 0.01, -18, 18)    
    def callback(self, ModelState):
        self.last_x = self.x
        self.last_y = self.y
        self.last_time = self.time_now
        
        if not math.isnan(ModelState.pose.position.x) and not math.isnan(ModelState.pose.position.y):
            self.x = ModelState.pose.position.x
            self.y = ModelState.pose.position.y
        else:
            print("x,y,vx,vy", self.x, self.y,self.vx,self.vy)
            print("x_error_sum,y_error_sum,vx_error_sum,vy_error_sum",self.control_x.error_sum,self.control_y.error_sum,self.control_vx.error_sum,self.control_vy.error_sum)
            rospy.logwarn("Received NaN values in ModelState data")
            return

        get_time = rospy.Time.now()
        self.time_now = get_time.to_sec()
        delta_time = self.time_now - self.last_time      
        
        self.vx = ModelState.twist.linear.x
        self.vy = ModelState.twist.linear.y 
    def reset_controls(self):
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.angular.z = 0
        self.control_x.error = 0  
        self.control_x.last_error = 0
        self.control_x.error_sum = 0
        self.control_y.error = 0  
        self.control_y.last_error = 0
        self.control_y.error_sum = 0
        self.control_vx.error = 0  
        self.control_vx.last_error = 0
        self.control_vx.error_sum = 0
        self.control_vy.error = 0  
        self.control_vy.last_error = 0
        self.control_vy.error_sum = 0

    def calculate_pid_outputs(self, error_x, error_y):
        temp_x = self.control_x.get_output(error_x)                 
        temp_y = self.control_y.get_output(error_y)
        error_vx = temp_x - self.vx
        error_vy = temp_y - self.vy
        self.twist.linear.x = self.control_vx.get_output(error_vx)
        self.twist.linear.y = self.control_vy.get_output(error_vy)
        self.twist.angular.z = 0


    def velocity_publisher(self):
        previous_state = 0
        rate = rospy.Rate(10)
        state = 0   
        rate.sleep()
        while not rospy.is_shutdown():
            print("x,y", self.x, self.y)
            length = len([-1.9, -2.5, -3.0, -3.3, -3.3, -3.5, -3.9, -4.0, -4.0, -3.9, -3.8, -3.5, -3.5, -3.3, -3.2, -3.3, -3.5, -3.5, -3.0, -2.7, -2.6, -2.8, -3.1, -3.3, -3.7, -3.9, -4.0, -4.0, -3.9, -3.5, -3.3, -3.3, -3.3, -3.3, -3.3, -2.9, -2.8, -2.4, -2.0, -1.7, -1.5, -1.4, -1.2, -1.1, -0.9, -0.9, -1.1, -1.3, -1.5, -1.7, -1.8, -1.9, -2.0, -2.0, -1.8, -1.7, -1.4, -0.8, -1.6, -1.9, -2.1, -2.1, -2.0, -1.3, -0.7, -0.4, -0.2, -0.2, -1.1, -1.5, -1.8, -1.9, -1.1, -0.2, 0.2, -0.7, -0.6, -0.6, -0.7, -0.8, -1.0, -1.2, -1.2, -1.1, -1.7, -1.1, -0.2, 0.2, 0.4, 0.5, 0.2, 0.5, 1.0, 1.0, 0.9, 0.8, 0.4, 0.3, 0.2, 1.3, 2.2, 2.4, 3.7, 4.0, 3.9, 3.6, 3.7, 3.7, 3.5, 3.2, 2.8, 2.3, 1.5, 1.0, 0.7, 0.5, 1.0, 1.0, 1.4, 1.9, 1.5, 2.2, 1.4, 0.8, 0.5, 1.0, 0.8])
            target_x, target_y = [-2.0, -2.4, -2.6, -2.8, -3.3, -3.6, -3.7, -3.3, -2.4, -1.7, -1.4, -1.5, -1.8, -2.1, -2.7, -2.1, -1.8, -1.5, -1.2, -0.3, 0.6, 1.5, 2.1, 2.3, 1.8, 2.3, 2.8, 3.5, 3.9, 4.0, 3.6, 3.2, 2.8, 2.3, 3.2, 3.2, 3.4, 3.1, 2.7, 2.9, 2.8, 2.4, 1.8, 1.3, 0.9, 0.7, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 2.6, 2.7, -2.0, -2.1, -2.2, -2.4, -2.4, -2.4, -2.4, -2.6, -3.0, -3.6, -3.6, -3.1, -2.6, -2.3, -0.1, -0.1, -0.1, -0.1, -0.1, -2.3, -2.3, 0.2, 0.5, 1.1, 1.6, 2.0, 2.3, 2.6, 2.8, 3.3, 2.9, 3.3, 3.0, 2.4, 1.6, -0.4, -2.3, -2.3, -0.2, 0.5, 1.2, 2.1, 2.4, 2.5, 2.5, 2.9, 3.7, 3.8, 1.8, 1.2, 0.1, -0.1, -1.0, -1.8, -2.2, -2.7, -3.2, -3.6, -3.9, -3.0, -2.5, -2.3, -0.3, 0.5, 0.5, 1.0, 1.5, 2.2, 2.9, 3.4, 2.6, 2.8, 3.4][state], [-1.9, -2.5, -3.0, -3.3, -3.3, -3.5, -3.9, -4.0, -4.0, -3.9, -3.8, -3.5, -3.5, -3.3, -3.2, -3.3, -3.5, -3.5, -3.0, -2.7, -2.6, -2.8, -3.1, -3.3, -3.7, -3.9, -4.0, -4.0, -3.9, -3.5, -3.3, -3.3, -3.3, -3.3, -3.3, -2.9, -2.8, -2.4, -2.0, -1.7, -1.5, -1.4, -1.2, -1.1, -0.9, -0.9, -1.1, -1.3, -1.5, -1.7, -1.8, -1.9, -2.0, -2.0, -1.8, -1.7, -1.4, -0.8, -1.6, -1.9, -2.1, -2.1, -2.0, -1.3, -0.7, -0.4, -0.2, -0.2, -1.1, -1.5, -1.8, -1.9, -1.1, -0.2, 0.2, -0.7, -0.6, -0.6, -0.7, -0.8, -1.0, -1.2, -1.2, -1.1, -1.7, -1.1, -0.2, 0.2, 0.4, 0.5, 0.2, 0.5, 1.0, 1.0, 0.9, 0.8, 0.4, 0.3, 0.2, 1.3, 2.2, 2.4, 3.7, 4.0, 3.9, 3.6, 3.7, 3.7, 3.5, 3.2, 2.8, 2.3, 1.5, 1.0, 0.7, 0.5, 1.0, 1.0, 1.4, 1.9, 1.5, 2.2, 1.4, 0.8, 0.5, 1.0, 0.8][state]
            error_x = target_x - self.x
            error_y = target_y - self.y

            if abs(error_x) + abs(error_y)< self.threshold_distance:
                previous_state = state
                state = (state + 1) % length
                if state == 0 and previous_state != 0:
                    rospy.loginfo("Completed final state, exiting.")
                    sys.exit(0)  # Exit the program when state wraps from length-1 to 0
                self.reset_controls()
            else:
                self.calculate_pid_outputs(error_x, error_y)

            self.pub.publish(self.twist) 
            rospy.loginfo("state: %0.1f",state)
            rate.sleep()
        sys.exit(0)

if __name__ == '__main__':
    try:
        rospy.init_node('controller',anonymous=True)
        vel = controller()
        vel.velocity_publisher()
    except rospy.ROSInterruptException:
        pass