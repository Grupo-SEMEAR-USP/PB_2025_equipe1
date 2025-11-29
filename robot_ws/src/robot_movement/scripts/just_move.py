#!/usr/bin/env python3
import rospy
import threading
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

class JustMove:
    def __init__(self):
        rospy.init_node('just_move', anonymous=True)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        # Variáveis da odometria
        self.last_yaw = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.first_run = True

        # Acumuladores
        self.distance_traveled = 0.0
        self.angle_turned = 0.0

        # Configuração da pista
        self.x_dist = 1.0  
        self.y_dist = 1.0  
        self.curve_radius = 0.75 

        # Sentido da rotação
        self.clockwise_mode = False 

        # Velocidades
        self.linear_vel = 0.2     # m/s (Nas retas)
        self.yaw_vel = 0.4        # rad/s (Nas curvas)

        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()

    def normalize_angle(self, angle):
        """ Mantém o ângulo entre -PI e PI para evitar erros na soma """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_cb(self, msg):
        # Pega a posição
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # Pega a orientação
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        curr_yaw = euler[2]

        if self.first_run:
            self.last_x = curr_x
            self.last_y = curr_y
            self.last_yaw = curr_yaw
            self.first_run = False
            return

        # Variações
        dx = curr_x - self.last_x
        dy = curr_y - self.last_y
        dist_inc = math.sqrt(dx**2 + dy**2) # Distância Euclidiana
        
        dyaw = curr_yaw - self.last_yaw
        dyaw = self.normalize_angle(dyaw)

        # Acumula
        self.distance_traveled += dist_inc
        self.angle_turned += dyaw

        # Atualiza para a próxima atualização
        self.last_x = curr_x
        self.last_y = curr_y
        self.last_yaw = curr_yaw

    def movement_loop(self):
        # Espera odometria iniciar
        while self.first_run and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.sleep(1.0)

        while not rospy.is_shutdown():
            # Avança em x
            self.move_straight(self.x_dist)
            
            # Gira em 90º (1ª)
            self.move_turn_90()
            
            # Avança em Y 
            self.move_straight(self.y_dist)
            
            # Gira em 90º (2ª)
            self.move_turn_90()

            # Retorna em x
            self.move_straight(self.x_dist)
            
            # Gira em 90º (3ª)
            self.move_turn_90()
            
            # Retorna em y
            self.move_straight(self.y_dist)
            
            # Gira em 90º (4ª)
            self.move_turn_90()

    def move_straight(self, distance):
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0
        
        self.distance_traveled = 0.0 # Reseta acumulador
        
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.distance_traveled >= distance:
                break
            self.pub.publish(cmd)
            rate.sleep()

    def move_turn_90(self):
        cmd = Twist()
        
        # Define sentido de giro
        if self.clockwise_mode:
            target_omega = -abs(self.yaw_vel)
        else:
            target_omega = abs(self.yaw_vel)

        # Calcula a velocidade linear necessária para manter o raio de 0.75m
        target_linear = abs(target_omega * self.curve_radius)

        cmd.linear.x = target_linear
        cmd.angular.z = target_omega
        
        self.angle_turned = 0.0 # Reseta acumulador
        target_angle = math.pi / 2.0 # 90 graus

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # Verifica se já girou 90 graus (em módulo)
            if abs(self.angle_turned) >= target_angle:
                break
            self.pub.publish(cmd)
            rate.sleep()

    def stop(self, duration):
        cmd = Twist()
        self.pub.publish(cmd) # Manda zeros
        rospy.sleep(duration)

if __name__ == "__main__":
    try:
        JustMove()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass