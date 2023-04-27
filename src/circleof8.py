#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi


class Fig8(): 
    def callback_function(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        self.x = pos_x
        self.y = pos_y
        # do abs(yaw) instead of just yaw to get rid of the instantaneous jump

        if self.startup: 
            self.starttime = rospy.get_rostime().secs
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y

        if(abs(self.x - self.x0) < 0.05) and (abs(self.y - self.y0) < 0.05) and (rospy.get_rostime().secs > self.starttime + 20):
            if self.reverse:
                print("stopping")
                #has already completed one loop, should stop moving
                self.ctrl_c = True
            else:
                print("reversing")
                self.starttime = rospy.get_rostime().secs
                self.reverse = True



    def __init__(self): 
        self.node_name = "odom_publisher" 
        topic_name = "cmd_vel" 

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function) #here

        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.startup = True
        self.reverse = False #here

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1# m/s
            if self.reverse:
                vel_cmd.angular.z = 0.2# rad/s
            else:
                vel_cmd.angular.z = -0.2# rad/s
            
            self.pub.publish(vel_cmd)
            self.rate.sleep()

        if self.ctrl_c:
            vel_cmd.linear.x = 0.0# m/s
            vel_cmd.angular.z = 0.0# rad/s
            self.pub.publish(vel_cmd)

       

if __name__ == '__main__': 
    publisher_instance = Fig8() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
