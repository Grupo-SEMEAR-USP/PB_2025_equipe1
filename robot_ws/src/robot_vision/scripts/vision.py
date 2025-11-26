import rospy
import cv2
import numpy as np
import threading
from collections import deque

from robot_communication.msg import vision_pattern



# --- Carregando Parâmetros de Configuração --- #
try:
    NWINDOWS = rospy.get_param('vision_params/detection/nwindows')
    MARGIN = rospy.get_param('vision_params/detection/margin')
    MINPIX = rospy.get_param('vision_params/detection/minpix')
    CAMERA_INDEX = rospy.get_param('vision_params/camera/index')
    S_TRESH = tuple(rospy.get_param('vision_params/thresholds/binary_threshold'))
    SOBEL_TRESH = tuple(rospy.get_param('vision_params/thresholds/sobel_threshold'))
    SRC_PC= rospy.get_param('vision_params/perspective/src_pc')
    MAX_CENTER_OFFSET_PIXELS = rospy.get_param('vision_params/normalization_parameters/max_offset')
    MIN_CURVATURE_RADIUS = rospy.get_param('vision_params/normalization_parameters/min_curvature')
    MAX_CURVATURE_RADIUS = rospy.get_param('vision_params/normalization_parameters/max_curvature')
    NODE_RATE_HZ = rospy.get_param('vision_params/node_config/rate')

except Exception as e:
    rospy.logwarn(f"Erro ao carregar parâmetros de configuração: {e}... Carregando valores padrão.")
    NWINDOWS = 9
    MARGIN = 100
    MINPIX = 50
    CAMERA_INDEX = 0
    S_TRESH = (160, 255)
    SOBEL_TRESH = (20, 100)
    SRC_PC= [
        (0.15, 0.55),
        (0.75, 0.55),
        (1.0, 1.0),
        (0.0, 1.0)
    ]
    MAX_CENTER_OFFSET_PIXELS = 300 
    MIN_CURVATURE_RADIUS = 300 
    MAX_CURVATURE_RADIUS = 5000
    NODE_RATE_HZ = 20


class RobotVision:
    def __init__(self):
        # Inicialização dos parâmetros e subscritores/publicadores ROS
        self.vision_pub = rospy.Publisher('/vision', vision_pattern, queue_size=10)

        self.queue_size = int(0.5 * NODE_RATE_HZ) # 5 amostras
        self.vision_queue = deque(maxlen=self.queue_size)

        # Variáveis internas
        self.cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2) # Garante backend V4L2
        self.Minv = None
        self.binary_warped_img = None
        self.warped_img = None
        self.cam_img = None
        self.left_fit = None 
        self.right_fit = None 
        self.avg_offset = 0.0
        self.avg_curvature = 0.0
        #definindo dados de visão
        self.vision_data = {
            'center_distance': 0.0,
            'curvature_radius': 0.0,
        }

        self.height, self.width = self.cam.read()[1].shape[:2]

        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()
        self.pub_thread = threading.Thread(target=self.publishing_loop, daemon=True)
        self.pub_thread.start()

    def publishing_loop(self):
        # Define a taxa de publicação
        rate = rospy.Rate(NODE_RATE_HZ) 

        while not rospy.is_shutdown():
            if len(self.vision_queue) > 0:
                
                # 1. Calcular a Média
                # Pega a média de todos os 'center_distance' na fila
                avg_offset = np.mean([d['center_distance'] for d in self.vision_queue])
                # Pega a média de todos os 'curvature_radius' na fila
                avg_curvature = np.mean([d['curvature_radius'] for d in self.vision_queue])
                
                # 2. Normalizar os Valores
                norm_offset = self.normalize_offset(avg_offset)
                norm_curvature = self.normalize_curvature(avg_curvature)
                self.avg_offset = norm_offset
                self.avg_curvature = norm_curvature
                # 3. Publicar
                vision_msg = vision_pattern()
                vision_msg.offset = norm_offset
                vision_msg.curvature = norm_curvature
                self.vision_pub.publish(vision_msg)
            
            rate.sleep()

    def camera_loop(self):
        if not self.cam.isOpened():
          rospy.logwarn("Erro fatal: Não foi possível abrir nenhuma câmera. Verifique as conexões.")
          return

        # Logo após abrir a câmera e antes do loop principal do ROS:
        for i in range(20):
            ret, frame = self.cam.read()
            # Apenas lendo para dar tempo ao sensor se ajustar à luz
    
        while not rospy.is_shutdown():
            ret, frame = self.cam.read()
            if rospy.get_param('vision_params/camera/flip'):  frame = cv2.flip(frame, -1)  # Gira a imagem 180 graus se necessário
            self.process_frame(frame)
            
            self.cam_img = frame
            if not ret:
                rospy.logwarn("Erro: Não foi possível ler o frame da câmera.")
                break

            self.vision_queue.append({
            'center_distance': self.vision_data['center_distance'],
            'curvature_radius': self.vision_data['curvature_radius']
            })
            img = self.debug_camera(frame)
            if rospy.get_param('vision_params/node_config/debug_mode'):
                cv2.imshow("a", img)
            cv2.waitKey(1)


        self.cam.release()

    def process_frame(self, frame):
        # Processamento do frame para detecção de faixas
        img_warped = self.perspective_transform(frame)

        binary_warped = self.binary_threshold(img_warped)


        left_fit_normalized, right_fit_normalized , y_mean = self.sliding_window_search(binary_warped)

        # --- DESNORMALIZAÇÃO ---
        A_prime_L, B_prime_L, C_prime_L = left_fit_normalized
        A_prime_R, B_prime_R, C_prime_R = right_fit_normalized

        # Coeficientes para a faixa ESQUERDA (left_fit)
        A_L = A_prime_L
        B_L = B_prime_L - 2 * A_prime_L * y_mean
        C_L = C_prime_L - B_prime_L * y_mean + A_prime_L * (y_mean**2)
        
        # Coeficientes para a faixa DIREITA (right_fit)
        A_R = A_prime_R
        B_R = B_prime_R - 2 * A_prime_R * y_mean
        C_R = C_prime_R - B_prime_R * y_mean + A_prime_R * (y_mean**2)
        
        # --- ARMAZENANDO OS COEFICIENTES ORIGINAIS ---
        self.left_fit = np.array([A_L, B_L, C_L])
        self.right_fit = np.array([A_R, B_R, C_R]) 
        # ----------------------------------------------

        curvature_radius = self.calculate_curvature(self.left_fit, self.right_fit, binary_warped.shape[0])
        center_distance = self.calculate_center_distance(self.left_fit, self.right_fit, binary_warped.shape[1])

        self.warped_img = img_warped
        self.binary_warped_img = binary_warped
        
        self.vision_data['center_distance'] = center_distance
        self.vision_data['curvature_radius'] = curvature_radius

    def binary_threshold(self, img):
        # Aplicar limiarização binária na imagem
        #Isola os pixels das faixas usando limiares de cor e gradiente.
        #Retorna uma imagem binária (preto e branco).

        #Converte para HLS (Hue, Lightness, Saturation) de acordo com o gpt é melhor que RGB para detecção de faixas
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        s_channel = hls[:, :, 2]  # Canal de Saturação
        
        # Binarização por Cor (bom para faixas amarelas)
        s_binary = cv2.inRange(s_channel, S_TRESH[0], S_TRESH[1])
        
        # Binarização por Gradiente (bom para faixas brancas)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Gradiente Sobel na direção X
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) 
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))

        sobel_binary = cv2.inRange(scaled_sobel, SOBEL_TRESH[0], SOBEL_TRESH[1])

        # Combinar os dois métodos
        # (Ou a faixa é detectada pela cor OU pelo gradiente)
        combined_binary = np.zeros_like(sobel_binary)
        combined_binary[(s_binary == 255) | (sobel_binary == 255)] = 255
        
        return combined_binary
    
    def perspective_transform(self, img):
        # Aplicar transformação de perspectiva
        h, w = img.shape[:2]

        TL = SRC_PC[0]
        TR = SRC_PC[1]
        BR = SRC_PC[2]
        BL = SRC_PC[3]

        src = np.float32([
            (w * TL[0], h * TL[1]),  # Top-left
            (w * TR[0], h * TR[1]),  # Top-right
            (w * BR[0], h * BR[1]),  # Bottom-right
            (w * BL[0], h * BL[1])   # Bottom-left
        ])

        dst = np.float32([
            (0, 0),        # Top-left
            (w , 0),        # Top-right
            (w , h),        # Bottom-right
            (0, h)         # Bottom-left
        ])
    
        # Faz as matrizes de transformação()
        M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)  # Matriz inversa para "desfazer" o warp depois
        warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

        return warped

    def sliding_window_search(self, binary_warped):
        # Implementar o algoritmo de janelas deslizantes
        h, w = binary_warped.shape

        histogram = np.sum(binary_warped[h//2:, :], axis=0)
        midpoint = np.int32(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        window_height = np.int32(h // NWINDOWS)

        # Identificar os pixels não zero na imagem binária
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        # Percorrer as janelas
        for window in range(NWINDOWS):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            win_xleft_low = leftx_current - MARGIN
            win_xleft_high = leftx_current + MARGIN
            win_xright_low = rightx_current - MARGIN
            win_xright_high = rightx_current + MARGIN

            # Identificar os pixels dentro da janela
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # Recentralizar a janela se muitos pixels forem encontrados
            if len(good_left_inds) > MINPIX:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > MINPIX:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass

        return self.average_slope_intercept(binary_warped, left_lane_inds, right_lane_inds)
    
    def average_slope_intercept(self, binary_warped, left_lane_inds, right_lane_inds):
        # Calcular a média dos coeficientes das linhas detectadas
        h, w = binary_warped.shape

        # Encontrar os pixels que pertencem às faixas
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # --- INÍCIO DA NORMALIZAÇÃO (CENTRALIZAÇÃO) ---

        # 1. Calcular a média de Y
        # Usamos np.mean(nonzeroy) ou h / 2, dependendo do que for mais representativo.
        # Usar a média de todos os pontos Y é geralmente mais robusto.
        if len(nonzeroy) > 0:
            y_mean = np.mean(nonzeroy)
        else:
            y_mean = h / 2 # Valor de fallback

        # 2. Subtrair a média dos dados Y (Centralização)
        lefty_normalized = lefty - y_mean
        righty_normalized = righty - y_mean

        # --- FIM DA NORMALIZAÇÃO ---

        # Ajustar uma linha polinomial de 2º grau (quadrática) aos pixels das faixas
        try:
            # Usamos os dados Y normalizados para o ajuste, mas os dados X originais
            left_fit_normalized = np.polyfit(lefty_normalized, leftx, 2)
            right_fit_normalized = np.polyfit(righty_normalized, rightx, 2)

        except (np.linalg.LinAlgError, ValueError, TypeError):
            # Em caso de erro, retornar coeficientes que definem uma linha horizontal nula
            left_fit_normalized = np.array([0, 0, 0])
            right_fit_normalized = np.array([0, 0, 0])

        # O resultado retornado (left_fit_normalized, right_fit_normalized)
        # são os coeficientes ajustados para os dados centralizados.
        # Você deve usá-los no resto do seu pipeline, e **lembrar-se** de que
        # eles são válidos APENAS para coordenadas Y que foram subtraídas por `y_mean`.
        
        return left_fit_normalized, right_fit_normalized , y_mean
    
    def calculate_curvature(self, left_fit, right_fit, img_height):
        # Calcular o raio de curvatura das faixas

        # Fórmula do raio de curvatura
        if(left_fit[0] == 0 or right_fit[0] == 0):
            return float('inf')
        left_curverad = ((1 + (2*left_fit[0]*img_height + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
        right_curverad = ((1 + (2*right_fit[0]*img_height + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

        # Retorna a média dos dois raios
        return (left_curverad + right_curverad) / 2
    
    def calculate_center_distance(self, left_fit, right_fit, img_width):
        # Calcular a distância do centro do robô ao centro da faixa

        try:
            # Posição da faixa esquerda e direita na base da imagem
            left_lane_base = left_fit[0]*(img_width)**2 + left_fit[1]*(img_width) + left_fit[2]
            right_lane_base = right_fit[0]*(img_width)**2 + right_fit[1]*(img_width) + right_fit[2]

            # Centro da faixa
            lane_center = (left_lane_base + right_lane_base) / 2.0

            # Centro da imagem (posição do robô)
            image_center = (img_width) / 2.0

            # Distância do centro do robô ao centro da faixa
            center_distance = image_center - lane_center
        except (TypeError, ValueError, np.linalg.LinAlgError):
            center_distance = 0.0
        
        return center_distance

    def debug_camera(self,frame):
        # Esta função visa apenas processar e retornar a imagem marcada.
        # A visualização (cv2.imshow e cv2.waitKey) será tratada no camera_loop.
        
        # 1. Obter dados
        left_fit = self.left_fit
        right_fit = self.right_fit 
        cam_img = frame
        binary_warped_img = self.binary_warped_img
        Minv = self.Minv
        avg_curvature = self.avg_curvature
        avg_offset = self.avg_offset

        # Verifica se as imagens essenciais foram inicializadas
        if cam_img is None or binary_warped_img is None or Minv is None:
            # Retorna a imagem original se os dados não estiverem prontos
            return np.zeros((self.height, self.width, 3), dtype=np.uint8) if cam_img is None else cam_img

        h, w = cam_img.shape[:2]
        ploty = np.linspace(0, h - 1, h)
        
        # 2. Calcular as posições X das faixas
        try:
            # Calcula os pixels X para cada Y baseado nos coeficientes polinomiais
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        except (TypeError, IndexError, ValueError):
            # Se 'fit' falhar (for None, tamanho incorreto, ou erro de cálculo)
            left_fitx = ploty * 0
            right_fitx = ploty * 0
            
        # 3. Criar a sobreposição do polígono da faixa
        # Usa a imagem warped binária como base de tamanho (mesmo que não seja usada diretamente)
        warp_zero = np.zeros_like(binary_warped_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Formatar os pontos para cv2.fillPoly()
        # Transpõe (x, y) e inverte o lado direito para criar um polígono fechado
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Desenhar o polígono da faixa (em verde)
        cv2.fillPoly(color_warp, np.int32([pts]), (0, 255, 0))

        # 4. "Desfazer" o warp da perspectiva e sobrepor
        new_warp = cv2.warpPerspective(color_warp, Minv, (w, h))
        
        # Combinar a imagem original com o polígono da faixa
        img_marcada = cv2.addWeighted(cam_img, 1, new_warp, 0.3, 0)

        # 5. Adicionar o texto (dados) na imagem
        curvatura = avg_curvature
        offset = avg_offset
        
        cv2.putText(img_marcada, f'Raio de Curvatura: {curvatura:8.0f}', 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        offset_dir = "Esquerda" if offset < 0 else "Direita"
        if abs(offset) < 1.0: offset_dir = "Centro"
        if offset == 0.0: offset_dir = "N/D"
            
        cv2.putText(img_marcada, f'Offset do Centro: {abs(offset):.2f} ({offset_dir})', 
                    (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # 6. Retorna a imagem marcada para ser exibida externamente
        return img_marcada
    
    def normalize_offset(self, offset_pixels):
        clamped_offset = np.clip(offset_pixels, -MAX_CENTER_OFFSET_PIXELS, MAX_CENTER_OFFSET_PIXELS)
        return clamped_offset / (MAX_CENTER_OFFSET_PIXELS*0.1)

    def normalize_curvature(self, curverad):
        # Normaliza o raio de curvatura para o intervalo [-1.0, 1.0].
        #     -1.0: Curvatura máxima para a esquerda.
        #      0.0: Linha reta (infinito).
        #     +1.0: Curvatura máxima para a direita.
        
        # 1. Calcula a curvatura inversa (1/radio)
        if curverad > MAX_CURVATURE_RADIUS: 
            inverse_curv = 0.0
        else:
            inverse_curv = 1.0 / curverad

        # 2. Assume que a curvatura é simétrica (por enquanto) e clampa a curvatura inversa.
        # Definir a curvatura máxima esperada. 1/MIN_CURVATURE_RADIUS
        MAX_INVERSE_CURVATURE = 1.0 / MIN_CURVATURE_RADIUS 

        clamped_inv_curv = np.clip(inverse_curv, -MAX_INVERSE_CURVATURE, MAX_INVERSE_CURVATURE)
        
        # Normaliza o valor inverso para -1.0 a 1.0
        if MAX_INVERSE_CURVATURE == 0.0:
            return 0.0
        norm_abs_curvature = clamped_inv_curv / (MAX_INVERSE_CURVATURE*0.1)
        
        # RETORNO TEMPORÁRIO (APENAS MAGNITUDE)
        return norm_abs_curvature

if __name__ == '__main__':
    try:
        rospy.init_node('vision', anonymous=True)
        rv = RobotVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

   
