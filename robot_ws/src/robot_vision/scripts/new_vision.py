#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import threading
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_communication.msg import vision_pattern 

class SimpleVision:
    def __init__(self):
        rospy.init_node('new_vision', anonymous=True)
        
        self.load_parameters()
        
        self.pub = rospy.Publisher('/vision', vision_pattern, queue_size=10)
        self.bridge = CvBridge()
        
        # Variáveis de estado
        self.latest_depth = None
        self.width = 640  # Valor padrão, atualiza no primeiro frame
        self.height = 480
        
        # Buffer para suavizar o offset
        self.offset_buffer = deque(maxlen=5)
        
        # Subscreve aos tópicos da RealSense
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)

    # Carrega parâmetros ou usa defaults seguros
    def load_parameters(self):
        try:
            self.s_thresh = tuple(rospy.get_param('vision_params/thresholds/binary_threshold', (90, 255)))
            self.sobel_thresh = tuple(rospy.get_param('vision_params/thresholds/sobel_threshold', (130, 255)))
            self.src_pc = rospy.get_param('vision_params/perspective/src_pc', [[0.2, 0.4], [0.8, 0.4], [1.0, 1.0], [0.0, 1.0]])
            self.track_width = rospy.get_param('vision_params/detection/track_width_px', 600)
            self.debug = rospy.get_param('vision_params/node_config/debug_mode', {'bin_img': False})
        except:
            rospy.logwarn("Erro carregando params. Usando defaults.")
            self.s_thresh = (90, 255)
            self.sobel_thresh = (130, 255)
            self.src_pc = [[0.2, 0.4], [0.8, 0.4], [1.0, 1.0], [0.0, 1.0]]
            self.track_width = 600
            self.debug = {'bin_img': False}

    # Pega profundidade crua (em mm)
    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError:
            pass

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.height, self.width = frame.shape[:2]
            
            # Se não tiver profundidade ainda, cria uma matriz de zeros para não travar
            if self.latest_depth is None:
                depth = np.zeros((self.height, self.width), dtype=np.uint16)
            else:
                depth = self.latest_depth

            self.process_pipeline(frame, depth)

        except CvBridgeError:
            pass

    def process_pipeline(self, color_img, depth_img):
        # Transformada de Perspectiva (Bird's Eye View)
        warped, M, Minv = self.perspective_transform(color_img)
        
        # Binarização (Achar as linhas brancas/amarelas)
        binary = self.binary_threshold(warped)

        # Detecção de Linhas (Hough Probabilístico)
        # minLineLength: Linhas menores que isso são ignoradas
        # maxLineGap: Espaços permitidos dentro de uma linha
        lines = cv2.HoughLinesP(binary, 1, np.pi/180, 50, minLineLength=50, maxLineGap=100)

        # Análise das Linhas
        offset, intersection_dist = self.analyze_lines(lines, depth_img, Minv)

        # Publicação
        msg = vision_pattern()
        msg.offset = float(offset)
        msg.ci_dist = float(intersection_dist)
        msg.curvature = 0.0 # Descomente se não removeu do .msg ainda
        
        self.pub.publish(msg)

        # Debug Visual 
        if self.debug.get('bin_img', False):
            # Desenha linhas no binário para debug
            debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            try:
                cv2.imshow("Debug SimpleVision", debug_img)
                cv2.waitKey(1)
            except: pass

    def analyze_lines(self, lines, depth_img, Minv):
        """
        Separa linhas em:
        - Verticais (Esquerda/Direita) -> Calculam Offset
        - Horizontais -> Calculam Interseção
        """
        if lines is None:
            return 0.0, 5.0 # Sem linhas: Offset 0, Longe de obstáculo

        left_lines_x = []
        right_lines_x = []
        intersections_y = [] # Coordenadas Y das linhas horizontais (no warped)

        center_x = self.width // 2

        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calcula ângulo da linha
            angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
            angle = abs(angle)

            # Classificação
            if 20 < angle < 160: # Linha "em pé" (Vertical)
                # Pega a posição X na base da imagem (y = height) para o offset
                # Equação da reta: x = (y - b)/m
                if x2 != x1:
                    m = (y2 - y1) / (x2 - x1)
                    b = y1 - m * x1
                    if m != 0:
                        x_base = int((self.height - b) / m)
                        
                        if x_base < center_x:
                            left_lines_x.append(x_base)
                        else:
                            right_lines_x.append(x_base)
            
            else: # Linha "deitada" (Horizontal) - Interseção/Pare
                # Pega o Y médio da linha
                y_avg = (y1 + y2) // 2
                intersections_y.append(y_avg)

        # Cálculo do offset
        lane_center = center_x
        
        # Cenário 1: Viu as duas faixas
        if len(left_lines_x) > 0 and len(right_lines_x) > 0:
            l_pos = np.mean(left_lines_x)
            r_pos = np.mean(right_lines_x)
            lane_center = (l_pos + r_pos) / 2.0
        
        # Cenário 2: Viu só a esquerda -> Estima a direita
        elif len(left_lines_x) > 0:
            l_pos = np.mean(left_lines_x)
            lane_center = l_pos + (self.track_width / 2.0)
            
        # Cenário 3: Viu só a direita -> Estima a esquerda
        elif len(right_lines_x) > 0:
            r_pos = np.mean(right_lines_x)
            lane_center = r_pos - (self.track_width / 2.0)

        # Offset: Distância do centro da imagem ao centro da faixa
        # Negativo = Robô está à direita (tem que virar p/ esquerda)
        # Positivo = Robô está à esquerda
        raw_offset = lane_center - center_x
        
        # Suavização
        self.offset_buffer.append(raw_offset)
        smooth_offset = np.mean(self.offset_buffer)

        # Cálculo da Interseção (Distância)
        ci_dist = 5.0 # Valor padrão (longe)
        
        if len(intersections_y) > 0:
            # Pega a linha horizontal mais próxima da parte inferior (maior Y no warped)
            closest_y_warped = max(intersections_y)
            
            # Precisamos "desentortar" esse ponto para achar a coordenada na imagem original (Depth)
            # Ponto no warped: (center_x, closest_y_warped)
            point_warped = np.array([[[center_x, closest_y_warped]]], dtype=np.float32)
            point_orig = cv2.perspectiveTransform(point_warped, Minv)
            
            u = int(point_orig[0][0][0])
            v = int(point_orig[0][0][1])
            
            # Pega profundidade com segurança
            try:
                if 0 <= v < self.height and 0 <= u < self.width:
                    depth_val = depth_img[v, u]
                    if depth_val > 0:
                        ci_dist = depth_val / 1000.0 # mm para metros
            except:
                pass

        return smooth_offset, ci_dist

    def binary_threshold(self, img):
        # Conversão HLS e Threshold
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        s_channel = hls[:, :, 2]
        s_binary = cv2.inRange(s_channel, self.s_thresh[0], self.s_thresh[1])
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) 
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
        sobel_binary = cv2.inRange(scaled_sobel, self.sobel_thresh[0], self.sobel_thresh[1])

        combined = np.zeros_like(sobel_binary)
        combined[(s_binary == 255) | (sobel_binary == 255)] = 255
        return combined

    def perspective_transform(self, img):
        h, w = img.shape[:2]
        # Pega pontos normalizados do YAML e converte para pixels
        src = np.float32([
            (w * self.src_pc[0][0], h * self.src_pc[0][1]),
            (w * self.src_pc[1][0], h * self.src_pc[1][1]),
            (w * self.src_pc[2][0], h * self.src_pc[2][1]),
            (w * self.src_pc[3][0], h * self.src_pc[3][1])
        ])
        
        # Retângulo do tamanho da imagem
        dst = np.float32([(0, 0), (w, 0), (w, h), (0, h)])
        
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)
        return warped, M, Minv

if __name__ == '__main__':
    try:
        SimpleVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass