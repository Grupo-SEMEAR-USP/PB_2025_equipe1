#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from robot_communication.msg import vision_pattern 

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_realsense', anonymous=True)

        # --- Publishers ---
        self.pub_vision = rospy.Publisher('/vision', vision_pattern, queue_size=10)
        
        # --- Ferramentas ---
        self.bridge = CvBridge()
        
        # --- Estado ---
        self.latest_depth = None
        self.target_point_warped = None   # Ponto alvo na visão aérea (x, y)
        self.target_point_original = None # Ponto alvo na visão da câmera (u, v)
        self.detected_angle = 90.0
        
        # --- Configuração ---
        self.load_parameters()

        # --- Subscribers ---
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)

        rospy.loginfo("Vision Node: Lógica de Intersecção Centralizada Iniciada")

    def load_parameters(self):
        try:
            self.src_pc = rospy.get_param('vision_params/perspective/src_pc', 
                                        [[0.20, 0.30], [0.80, 0.30], [1.0, 1.0], [0.0, 1.0]])
            self.show_debug = rospy.get_param('vision_params/node_config/debug_mode/bin_img', False)
            self.track_width_px = 500 # Largura estimada da pista em pixels no warped (ajuste se necessário)
        except:
            self.src_pc = [[0.20, 0.30], [0.80, 0.30], [1.0, 1.0], [0.0, 1.0]]
            self.show_debug = False
            self.track_width_px = 500

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError: pass

    def image_cb(self, msg):
        try:
            # 1. Captura
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if rospy.get_param('vision_params/camera/flip', False):
                frame = cv2.flip(frame, -1)

            h, w = frame.shape[:2]
            image_center_x = w / 2.0

            # 2. Perspectiva (Essencial para medir offset linearmente)
            warped, M, Minv = self.perspective_transform(frame)

            # 3. Processamento Geométrico
            # Retorna offset (pixels), distancia Y da intersecção (pixels), e ângulo
            offset_px, inter_y_px, angle = self.process_geometry(warped)

            # 4. Mapeamento Reverso e Profundidade
            msg_vision = vision_pattern()
            msg_vision.offset = float(offset_px) # Offset calculado no warped é mais confiável
            msg_vision.curvature = float(angle)
            
            dist_real = 0.0

            # Se detectamos uma linha de intersecção (Y > 0)
            if inter_y_px > 0:
                # O ponto alvo é: (Centro da Imagem - Offset, Altura da Intersecção)
                # Ou seja: Onde o centro do robô cruza a linha de pare
                target_x_warped = image_center_x - offset_px 
                target_y_warped = inter_y_px
                
                self.target_point_warped = (int(target_x_warped), int(target_y_warped))

                # Transforma esse ponto de volta para a imagem original para ler o Depth
                point_warped_vec = np.array([[[float(target_x_warped), float(target_y_warped)]]], dtype=np.float32)
                point_orig_vec = cv2.perspectiveTransform(point_warped_vec, Minv)
                
                orig_x = int(point_orig_vec[0][0][0])
                orig_y = int(point_orig_vec[0][0][1])
                self.target_point_original = (orig_x, orig_y)

                # Lê a profundidade real
                dist_real = self.get_depth_at_point(orig_x, orig_y)
            
            else:
                self.target_point_warped = None
                self.target_point_original = None
                # Se não tem intersecção, distância é "longe" (ou 0 se preferir lógica de erro)
                dist_real = 0.0 

            msg_vision.ci_dist = float(dist_real)
            self.pub_vision.publish(msg_vision)

            # 5. Debug
            if self.show_debug:
                self.draw_debug(frame, warped, msg_vision)

        except CvBridgeError: pass

    def process_geometry(self, warped_frame):
        """
        Retorna: 
        - offset_px: Desvio lateral do centro da pista.
        - intersection_y: Altura da linha horizontal (0 se não houver).
        - angle: Ângulo predominante das linhas verticais.
        """
        h, w = warped_frame.shape[:2]
        center_x = w // 2

        # Máscara e Bordas
        mask = self.color_mask(warped_frame)
        edges = self.sobel_edges(mask)
        
        # Hough (Detecta linhas)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 40, minLineLength=30, maxLineGap=40)

        vertical_lines_x = [] # Guarda a posição X da base das linhas verticais
        horizontal_lines_y = [] # Guarda a posição Y das linhas horizontais
        angles = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calcula ângulo em graus (0 = Horizontal, 90 = Vertical)
                angle_rad = np.arctan2(abs(y2 - y1), abs(x2 - x1))
                angle_deg = np.degrees(angle_rad)

                # Classifica as linhas
                if angle_deg > 60: # Vertical (Faixa lateral)
                    # Projeta onde essa linha toca a base da imagem (y=h)
                    # x = x1 + (h - y1) / m
                    if y2 != y1: # Evita div por zero
                        slope = (y2 - y1) / (x2 - x1 + 1e-6)
                        if abs(slope) > 0.5:
                            x_base = x1 + (h - y1) / slope
                            vertical_lines_x.append(x_base)
                            angles.append(angle_deg if slope > 0 else -angle_deg) # Sinal do ângulo

                elif angle_deg < 20: # Horizontal (Intersecção)
                    y_avg = (y1 + y2) // 2
                    horizontal_lines_y.append(y_avg)

        # --- CÁLCULO DO OFFSET (Centro da Pista) ---
        lane_center = center_x # Default: Meio da imagem
        
        if len(vertical_lines_x) > 0:
            # Separa em esquerda e direita do centro atual
            lefts = [x for x in vertical_lines_x if x < center_x]
            rights = [x for x in vertical_lines_x if x > center_x]
            
            if len(lefts) > 0 and len(rights) > 0:
                lane_center = (np.mean(lefts) + np.mean(rights)) / 2.0
            elif len(lefts) > 0:
                lane_center = np.mean(lefts) + (self.track_width_px / 2.0)
            elif len(rights) > 0:
                lane_center = np.mean(rights) - (self.track_width_px / 2.0)
        
        # Offset: Distância do centro da faixa ao centro da imagem
        # Se lane_center (300) < center_x (320) -> Offset = -20 (Robô deve ir para esquerda? Não, offset negativo costuma ser erro p/ esquerda)
        # Padrão ROS: Erro = Target - Atual. 
        # Aqui: Offset = Lane_Center - Image_Center
        offset = lane_center - center_x

        # --- CÁLCULO DA INTERSECÇÃO (Y) ---
        inter_y = 0
        if len(horizontal_lines_y) > 0:
            # Pega a linha mais próxima (maior Y no warped = mais perto)
            inter_y = np.max(horizontal_lines_y)

        # --- ÂNGULO ---
        avg_angle = 90.0
        if len(angles) > 0:
            avg_angle = np.mean(angles)

        return offset, inter_y, avg_angle

    def get_depth_at_point(self, u, v):
        if self.latest_depth is None: return 0.0
        h, w = self.latest_depth.shape
        # Verifica limites e pega uma região 3x3 para evitar pixel morto
        if 2 <= v < h-2 and 2 <= u < w-2:
            roi = self.latest_depth[v-2:v+3, u-2:u+3]
            valid = roi[roi > 0]
            if len(valid) > 0:
                return np.median(valid) / 1000.0
        return 0.0

    # --- PROCESSAMENTO BÁSICO ---
    def perspective_transform(self, img):
        h, w = img.shape[:2]
        src = np.float32([
            [w * self.src_pc[0][0], h * self.src_pc[0][1]],
            [w * self.src_pc[1][0], h * self.src_pc[1][1]],
            [w * self.src_pc[2][0], h * self.src_pc[2][1]],
            [w * self.src_pc[3][0], h * self.src_pc[3][1]]
        ])
        dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)
        return warped, M, Minv

    def color_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Amarelo e Branco (Ajuste os limiares conforme sua pista)
        y_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([40, 200, 200]))
        w_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        mask = cv2.bitwise_or(y_mask, w_mask)
        kernel = np.ones((5,5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def sobel_edges(self, mask):
        sobelx = cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3)
        mag = np.sqrt(sobelx**2 + sobely**2)
        mag = np.uint8(255 * mag / np.max(mag)) if np.max(mag) > 0 else np.zeros_like(mask)
        _, binary = cv2.threshold(mag, 50, 255, cv2.THRESH_BINARY)
        return binary

    def draw_debug(self, original, warped, msg):
        try:
            h, w = original.shape[:2]
            center_x = w // 2
            
            # 1. Desenha no Warped (Lógica geométrica)
            # Centro da imagem
            cv2.line(warped, (center_x, 0), (center_x, h), (255, 0, 0), 1)
            # Centro da pista calculado (Offset)
            lane_center_x = int(center_x + msg.offset)
            cv2.line(warped, (lane_center_x, 0), (lane_center_x, h), (0, 255, 0), 2)
            
            # Ponto alvo no warped
            if self.target_point_warped:
                cv2.circle(warped, self.target_point_warped, 10, (0, 0, 255), -1)

            # 2. Desenha no Original (Realidade)
            if self.target_point_original:
                orig_pt = self.target_point_original
                cv2.circle(original, orig_pt, 10, (0, 0, 255), -1)
                cv2.putText(original, f"Dist: {msg.ci_dist:.2f}m", (orig_pt[0]+15, orig_pt[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.putText(original, f"Offset: {msg.offset:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Warped Analysis", warped)
            cv2.imshow("Real View", original)
            cv2.waitKey(1)
        except Exception: pass

if __name__ == '__main__':
    try:
        VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass