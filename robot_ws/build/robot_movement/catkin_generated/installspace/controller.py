#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from robot_communication.msg import vision_pattern

class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # Velocidades
        self.MAX_LINEAR_SPEED = 0.5  # m/s (Velocidade máxima na reta)
        self.MIN_LINEAR_SPEED = 0.2  # m/s (Velocidade mínima em curvas fechadas)
        
        # Ganhos do PID
        self.Kp = 0.8  # Proporcional: reage à intensidade do erro
        self.Ki = 0.0  # Integral: corrige erros acumulados (cuidado, pode oscilar)
        self.Kd = 0.1  # Derivativo: segura a oscilação, prevê o erro futuro
        
        # Ganho do Feedforward (Curvatura)
        # Se 1.0, confia 100% na matemática v = w*R.
        # Ajuste para mais ou menos se o robô sair muito ou pouco nas curvas.
        self.K_curvature = 1.0 

        # Variáveis de Controle
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()

        # Publishers e Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vision_sub = rospy.Subscriber('/vision', vision_pattern, self.control_loop)

        # Buffer para segurança
        self.twist_msg = Twist()

        rospy.loginfo("Lane Controller Iniciado. Aguardando dados de visão...")

    def control_loop(self, msg):
        """
        Recebe a msg vision_pattern:
        float32 curvature (Raio em metros)
        float32 offset (Erro lateral em metros)
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Proteção contra dt=0 na primeira execução
        if dt == 0:
            self.last_time = current_time
            return

        # Extração dos dados
        radius = msg.curvature
        offset = msg.offset # Assumindo: Negativo = Robô à esq, Positivo = Robô à dir

        # Proteção: Se raio for 0 ou muito pequeno (erro de visão), assume reta (infinito)
        if abs(radius) < 0.1:
            radius = 999.0 

        # Controle de Velocidade Linear (Dinâmico)
        # Quanto menor o raio (curva fechada), menor a velocidade
        linear_v = self.MAX_LINEAR_SPEED - (1.0 / abs(radius))
        linear_v = max(self.MIN_LINEAR_SPEED, min(linear_v, self.MAX_LINEAR_SPEED))

        # Cálculo do PID para o Offset (Feedback)
        error = offset # O setpoint é 0 (centro da faixa)
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        # Saída do PID (Angular Z)
        # NOTA: O sinal depende da sua câmera. Se offset positivo exige virar para esquerda (+Z), use +Kp.
        # Se offset positivo exige virar para direita (-Z), use -Kp.
        # Geralmente: Se estou à direita (offset > 0), devo virar à esquerda (+Z).
        angular_pid = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # 4. Cálculo do Feedforward da Curvatura (Física)
        # w = v / R
        # O sinal depende se a curva é para esquerda ou direita. 
        # Assumindo que o código de visão retorna Raio com sinal (Negativo esq / Positivo dir) ou vice-versa.
        # Vamos supor que o raio venha absoluto e precisamos saber o lado pelo contexto,
        # mas geralmente o código de visão deve enviar raio com sinal ou curvatura com sinal.
        # VOU ASSUMIR: Raio positivo = Curva Esquerda, Raio negativo = Curva Direita
        try:
            angular_ff = (linear_v / radius) * self.K_curvature
        except ZeroDivisionError:
            angular_ff = 0.0

        # Combinação dos Comandos
        angular_total = angular_pid + angular_ff

        # Preparar mensagem
        self.twist_msg.linear.x = linear_v
        self.twist_msg.angular.z = angular_total

        # Publicar
        self.cmd_vel_pub.publish(self.twist_msg)

        # Atualizar variáveis para próxima iteração
        self.prev_error = error
        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = Controller()
        node.run()
    except rospy.ROSInterruptException:
        pass