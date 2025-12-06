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
        
        # --- Variáveis de Estado ---
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.first_run = True
        self.ideal_yaw = 0.0 
        self.turn_approx = 89.0

        # --- Configuração da Pista ---
        self.x_dist = 2.04  
        self.y_dist = 2.015
        self.x_desloc = 0.0
        # self.x_desloc = 1.09 - 0.075
        self.curve_radius = 0.68 
        self.clockwise_mode = False 

        # --- Velocidades ---
        self.linear_vel = 0.4     
        self.yaw_vel = 0.3        

        # --- Ganhos do PID ---
        self.kp = 2     # Força da correção imediata
        self.ki = 0.15   # Corrige o erro de regime (puxada lateral)
        self.kd = 0.1   # Evita oscilação (segura a inércia)
        
        # --- Variáveis do PID ---
        self.integral_error = 0.0
        self.last_error = 0.0
        self.max_integral = 0.5 # Anti-windup (limite do acumulador)

        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def odom_cb(self, msg):
        # Apenas atualiza estado global. Cálculos ficam no loop principal.
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        
        # Variável de classe
        self.current_yaw = euler[2]

        if self.first_run:
            self.ideal_yaw = self.current_yaw 
            self.first_run = False

    def movement_loop(self):
        rospy.loginfo("Aguardando odometria...")
        while self.first_run and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # rospy.sleep(1.0)
        rospy.loginfo("Iniciando circuito!")
        
        start_dist = self.x_dist - self.x_desloc

        while not rospy.is_shutdown():
            # Inicia X com deslocamento
            rospy.loginfo("Primeira reta!")
            self.move_straight(start_dist)
            self.move_turn_approx_90()
            
            # Reta Y
            rospy.loginfo("Segunda reta!")
            self.move_straight(self.y_dist)
            self.move_turn_approx_90()

            # Volta X
            rospy.loginfo("Terceira reta!")
            self.move_straight(self.x_dist)
            self.move_turn_approx_90()
            
            # Volta Y
            rospy.loginfo("Quarta reta!")
            self.move_straight(self.y_dist)
            self.move_turn_approx_90()

            # Finaliza
            rospy.loginfo("Finalizando!")
            # self.move_straight(self.x_desloc)
            self.move_straight(self.x_dist)
            self.move_turn_approx_90()

    def move_straight(self, target_distance):
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        
        # Pega ponto inicial da reta (Reference Frame)
        start_x = self.current_x
        start_y = self.current_y
        
        # Reseta o PID para a nova reta
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # Target fixo matemático (Evita drift acumulado)
        target_angle = self.ideal_yaw

        # Controle de tempo para o PID (dt)
        last_time = rospy.Time.now()
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            
            if dt == 0: # Evita divisão por zero na primeira iteração
                rate.sleep()
                continue
            
            last_time = current_time

            # --- CÁLCULO DE DISTÂNCIA PROJETADA (Correção Vetorial) ---
            # Em vez de medir o caminho percorrido (que soma o zigue-zague),
            # medimos o quanto o robô avançou na direção do ângulo ideal.
            # Fórmula: Produto Escalar (Dot Product)
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            
            # Projeção no vetor diretor da pista
            dist_projected = dx * math.cos(target_angle) + dy * math.sin(target_angle)

            if dist_projected >= target_distance:
                break

            # Cálculo do PID
            error = self.normalize_angle(target_angle - self.current_yaw) # Sinal invertido corrigido aqui

            # Proporcional
            P = self.kp * error

            # Integral (Corrige o viés mecânico/chão torto)
            self.integral_error += error * dt
            # Anti-windup (Trava o integrador para não acumular infinito)
            self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
            I = self.ki * self.integral_error

            # Derivativo (Segura a oscilação)
            derivative = (error - self.last_error) / dt
            D = self.kd * derivative

            self.last_error = error

            # Saída do Controle
            # Nota: angular.z positivo gira para a esquerda no ROS
            # Se target (0) - current (-0.1) = +0.1 (erro positivo) -> Robô deve virar p/ esquerda
            cmd.angular.z = P + I + D

            self.pub.publish(cmd)
            rate.sleep()
        
        self.stop(0.5) # Pausa importante para zerar inércia

    def move_turn_approx_90(self):
        cmd = Twist()
        
        # Atualiza a referência IDEAL
        rotation_rad = (math.pi / 180.0) * self.turn_approx
        if self.clockwise_mode:
            self.ideal_yaw -= rotation_rad
            target_omega = -abs(self.yaw_vel)
        else:
            self.ideal_yaw += rotation_rad
            target_omega = abs(self.yaw_vel)
            
        self.ideal_yaw = self.normalize_angle(self.ideal_yaw)

        # Feedforward
        target_linear = abs(target_omega * self.curve_radius)
        cmd.linear.x = target_linear
        cmd.angular.z = target_omega
        
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # Erro em relação ao ângulo IDEAL FINAL
            error = self.normalize_angle(self.current_yaw - self.ideal_yaw)
            
            # Para a curva quando estiver alinhado com o próximo eixo
            if abs(error) < 0.03: # ~1.7 graus de tolerância
                break
                
            self.pub.publish(cmd)
            rate.sleep()
            
        self.stop(1.0)

    def stop(self, duration):
        cmd = Twist()
        self.pub.publish(cmd)
        rospy.sleep(duration)

if __name__ == "__main__":
    try:
        JustMove()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass