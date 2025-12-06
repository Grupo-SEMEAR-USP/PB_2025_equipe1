#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class LinearTest:
    def __init__(self):
        rospy.init_node('linear_calibration_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = None
        self.start_y = None
        self.distance_moved = 0.0
        self.is_moving = False

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.start_x is not None:
            # Calcula distância Euclidiana percorrida desde o início do teste
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            self.distance_moved = math.sqrt(dx**2 + dy**2)

    def run_test(self, target_distance_meters, speed_ms):
        # Espera conexão
        while self.pub.get_num_connections() == 0:
            rospy.sleep(0.1)
            
        rospy.sleep(1) # Estabiliza odom
        
        # Marca posição inicial
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.distance_moved = 0.0
        
        cmd = Twist()
        cmd.linear.x = speed_ms
        
        rospy.loginfo(f"--- INICIANDO TESTE ---")
        rospy.loginfo(f"Alvo: {target_distance_meters} metros")
        rospy.loginfo(f"Posição Inicial Odom: X={self.start_x:.2f}")

        rate = rospy.Rate(50) # 50Hz
        
        while not rospy.is_shutdown():
            if self.distance_moved >= target_distance_meters:
                break
            
            self.pub.publish(cmd)
            rate.sleep()
            
        # Parar o robô
        cmd.linear.x = 0.0
        self.pub.publish(cmd)
        
        rospy.loginfo(f"--- FIM ---")
        rospy.loginfo(f"O Robô 'acha' que andou: {self.distance_moved:.4f} metros")
        rospy.loginfo("Agora meça a distância REAL no chão com uma trena.")

if __name__ == "__main__":
    try:
        tester = LinearTest()
        # CONFIGURAÇÃO: 2 metros a 0.2 m/s
        tester.run_test(2.0, 0.2)
    except rospy.ROSInterruptException:
        pass