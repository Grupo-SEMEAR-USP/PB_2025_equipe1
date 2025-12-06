#!/usr/bin/env python3
import rospy
import threading
import numpy as np
from geometry_msgs.msg import Twist 
from robot_communication.msg import vision_pattern

class MovementController:
    def __init__(self):
        rospy.init_node('mov_controller_node')
        self.mov_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.mov_subscriber = rospy.Subscriber('/vision', vision_pattern, self.cb_vision)
        self.offset = -0.0
        self.curvature = 0.0
        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.start()

    def movement_loop(self):
        while not rospy.is_shutdown():
            linear_x, angular_z = self.convert_to_mov()
            self.update_cmd(linear_x, angular_z)
            rospy.sleep(0.1)
           

    def cb_vision(self, msg):
        self.offset = msg.offset
        self.curvature = msg.curvature
        pass
    
    def convert_to_mov(self):
        offset_gain = 0.2
        curvature_gain = 0.00005

        linear_x = abs(self.curvature) * 0.0005 -0.2
        angular_z = -self.offset * offset_gain + self.curvature * curvature_gain

        return linear_x, angular_z

    def update_cmd(self, linear_x, angular_z):
        mov_msg = Twist()
        mov_msg.linear.x = linear_x
        mov_msg.angular.z = angular_z
        self.mov_pub.publish(mov_msg)
        pass

if __name__ == '__main__':
    try:
        mov_controller = MovementController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass