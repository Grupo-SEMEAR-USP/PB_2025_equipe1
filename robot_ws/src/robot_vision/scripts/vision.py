import rospy
import pyrealsense2 as rs
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
    NODE_RATE_HZ = rospy.get_param('vision_params/node_config/rate')
    TRACK_WIDTH_PIXELS = rospy.get_param('vision_params/detection/track_width_px')
    RS_WIDTH = rospy.get_param('vision_params/camera/rs/width')
    RS_HEIGHT = rospy.get_param('vision_params/camera/rs/height')

except Exception as e:
    rospy.logwarn(f"Erro ao carregar parâmetros de configuração: {e}... Carregando valores padrão.")
    NWINDOWS = 9
    MARGIN = 100
    MINPIX = 50
    CAMERA_INDEX = 2
    S_TRESH = (160, 255)
    SOBEL_TRESH = (20, 100)
    SRC_PC= [
        (0.15, 0.55),
        (0.75, 0.55),
        (1.0, 1.0),
        (0.0, 1.0)
    ]
    NODE_RATE_HZ = 20
    TRACK_WIDTH_PIXELS = 600  


class RobotVision:
    def __init__(self):
        # Inicialização dos parâmetros e subscritores/publicadores ROS
        self.vision_pub = rospy.Publisher('/vision', vision_pattern, queue_size=10)

        self.queue_size = int(0.5 * NODE_RATE_HZ) # 5 amostras
        self.vision_queue = deque(maxlen=self.queue_size)

        if(CAMERA_INDEX==-1):
            self.realsense_init()
            self.height, self.width = [RS_HEIGHT,RS_WIDTH]
        else:
            self.cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2) # Garante backend V4L2
            self.height, self.width = self.cam.read()[1].shape[:2]
        
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
            'area_left': 0.0,
            'area_right': 0.0,
            'closest_intersection': (0,0),
            'closest_inter_dist': 0.0,
            'min_angle': 0.0,
            'min_angle_line': None,
            'lines': None,
            'norm_angle_hough': 0.0,
            'norm_offset_hough': 0.0
        }


        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()
        self.pub_thread = threading.Thread(target=self.publishing_loop, daemon=True)
        self.pub_thread.start()

    def realsense_init(self):
        # Configure depth and color streams
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()

        # Get device product line for setting a supporting resolution
        self.rs_pipeline_wrapper = rs.pipeline_wrapper(self.rs_pipeline)
        self.rs_pipeline_profile = self.rs_config.resolve(self.rs_pipeline_wrapper)
        self.rs_device = self.rs_pipeline_profile.get_device()
        self.rs_device_product_line = str(self.rs_device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.rs_device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.rs_pipeline.start(self.rs_config)
        
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
                self.avg_offset = avg_offset
                self.avg_curvature = avg_curvature
                # 3. Publicar
                vision_msg = vision_pattern()
                vision_msg.offset = avg_offset
                vision_msg.curvature = avg_curvature
                self.vision_pub.publish(vision_msg)
            
            rate.sleep()

    def camera_loop(self):  
        if(CAMERA_INDEX!=-1):
            if not self.cam.isOpened():
                rospy.logwarn("Erro fatal: Não foi possível abrir nenhuma câmera. Verifique as conexões.")
                return

            # Logo após abrir a câmera e antes do loop principal do ROS:
            for i in range(20):
                ret, frame = self.cam.read()
                self.process_frame(frame)

        while not rospy.is_shutdown():

            if CAMERA_INDEX!=-1:
                ret, frame = self.cam.read()

                if not ret:
                    rospy.logwarn("Erro: Não foi possível ler o frame da câmera.")
                    break
            else:
                # Wait for a coherent pair of frames: depth and color
                frames = self.rs_pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                frame = color_image


            if rospy.get_param('vision_params/camera/flip'):  frame = cv2.flip(frame, -1)  # Gira a imagem 180 graus se necessário
            self.process_frame(frame)
            
            sw_img , hg_img,bin_img = self.debug_camera(frame)
            if rospy.get_param('vision_params/node_config/debug_mode'):
                cv2.imshow("Sliding Windowns Detect", sw_img)
                cv2.imshow("Hough Detect", hg_img)
                cv2.imshow("Binary Image", bin_img)
            cv2.waitKey(1)


        self.cam.release()

    def process_frame(self, frame):
        # Processamento do frame para detecção de faixas
        img_warped = self.perspective_transform(frame)

        binary_warped = self.binary_threshold(img_warped)
        self.binary_warped_img = binary_warped

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

        hg_img = self.binary_threshold(frame.copy())
        cinter_dist ,lines, min_angle_line, closest_inter, min_angle, area_left, area_right, norm_angle, norm_offset_hough = self.hough_line_detection(hg_img)
        # --- ATUALIZAÇÃO DO DICIONÁRIO vision_data ---
        # Valores diretos retornados pela função atualizada
        
        curvature_radius = self.calculate_curvature(self.left_fit, self.right_fit, binary_warped.shape[0])
        center_distance = self.calculate_center_distance(self.left_fit, self.right_fit, binary_warped.shape[1])
        self.vision_data['center_distance'] = center_distance
        self.vision_data['curvature_radius'] = curvature_radius

        # Dados do Hough
        self.vision_data['area_left'] = area_left
        self.vision_data['area_right'] = area_right
        self.vision_data['closest_intersection'] = closest_inter
        if cinter_dist != None:
            self.vision_data['closest_inter_dist'] = cinter_dist
        self.vision_data['min_angle'] = min_angle
        self.vision_data['min_angle_line'] = min_angle_line
        self.vision_data['lines'] = lines
        
        # Novos campos para controle Hough
        self.vision_data['norm_angle_hough'] = norm_angle
        self.vision_data['norm_offset_hough'] = norm_offset_hough


        self.vision_queue.append({
        'center_distance': self.vision_data['center_distance'],
        'curvature_radius': self.vision_data['curvature_radius'],
        'closest_inter_dist': self.vision_data['closest_inter_dist'],
        })

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
        h, w = [self.height,self.width]

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

        h, w = [self.height,self.width]
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
        img_marcada_hough = frame.copy()
        lines = self.vision_data['lines']
        min_angle_line = self.vision_data['min_angle_line']
        closest_inter = self.vision_data['closest_intersection']
        min_angle = self.vision_data['min_angle']
        # Desenha a linha central
        center_x = w // 2
        cv2.line(img_marcada_hough, (center_x, 0), (center_x, h), (0, 255, 255), 2)

        # Desenha TODAS as linhas detectadas
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(img_marcada_hough, (x1, y1), (x2, y2), (255, 199, 209), 1) # Linhas leves (rosa claro)

            # Desenha as setas de deslocamento (usando o mesmo cálculo de área)
            step = 10
            for y in range(0, h, step):
                delta_xs = []
                for x1, y1, x2, y2 in lines[:, 0]:
                    if x2 != x1:
                        m = (y2 - y1) / (x2 - x1)
                        b = y1 - m * x1
                        if abs(m) > 1e-6:
                            x_at_y = int((y - b) / m)
                        else:
                            x_at_y = x1
                    else:
                        x_at_y = x1
                    
                    if 0 <= x_at_y < w:
                        delta_xs.append(x_at_y - center_x)

                if delta_xs:
                    min_dx = min(delta_xs, key=lambda dx: abs(dx))
                    x_min = center_x + min_dx
                    color = (0, 255, 0) if min_dx >= 0 else (0, 0, 255) # Verde para direita, Vermelho para esquerda
                    cv2.line(img_marcada_hough, (center_x, y), (x_min, y), color, 1)


        # Desenha a linha de Menor Ângulo e a Interseção (se encontradas)
        if min_angle_line is not None and closest_inter is not None:
            x1, y1, x2, y2 = min_angle_line
            cv2.line(img_marcada_hough, (x1, y1), (x2, y2), (0, 255, 255), 2) # Linha de destaque (amarelo)
            cv2.circle(img_marcada_hough, closest_inter, 6, (0, 255, 255), -1) # Círculo na interseção

            # Exibe o ângulo
            cv2.putText(img_marcada_hough, f"Menor angulo: {min_angle:.2f}°", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            # Exibe a interceção mais proxima
            cv2.putText(img_marcada_hough, f"Intersecção mais Proxima: ({(self.vision_data['closest_inter_dist']):.2f})", (10, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        img_bin = None
        try:
            img_bin = cv2.warpPerspective(binary_warped_img,Minv,(w,h))
        except Exception as e:
            rospy.logwarn(e)

        
        cv2.line(img_marcada, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)

        # Desenhar linha do CENTRO DA FAIXA (Onde o robô deveria estar) - VERMELHO
        # (Recalcula a posição baseada nos polinomios encontrados)
        try:
            lane_center_x = int((left_fit[0]*h**2 + left_fit[1]*h + left_fit[2] + 
                                right_fit[0]*h**2 + right_fit[1]*h + right_fit[2]) / 2)
            cv2.line(img_marcada, (lane_center_x, 0), (lane_center_x, h), (0, 0, 255), 2)
        except:
            pass
            
        return img_marcada , img_marcada_hough , img_bin
    
    def line_inter(self, line1, line2):
        tolerance = 1e-6
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        determ = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        if abs(determ) <= tolerance:
            px = (x3 + x4)/ 2
            py = y3
            return int(px), int(py)

        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / determ
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / determ

  
        
        return int(px), int(py)

    def get_roi_mask(self, img_shape, src_pc):
        """
        Cria uma máscara binária para a Região de Interesse (ROI) 
        baseada nos pontos normalizados de perspectiva (SRC_PC).
        """
        h, w = img_shape[:2]
        
        # Converte os pontos normalizados (0 a 1) para coordenadas de pixel
        TL = src_pc[0]
        TR = src_pc[1]
        BR = src_pc[2]
        BL = src_pc[3]

        # Define os vértices do polígono do ROI
        vertices = np.array([
            [(w * BL[0], h * BL[1]), 
             (w * BR[0], h * BR[1]), 
             (w * TR[0], h * TR[1]),
             (w * TL[0], h * TL[1])]], 
            dtype=np.int32)
        
        # Cria uma máscara preta
        mask = np.zeros(img_shape, dtype=np.uint8)
        
        # Preenche o polígono do ROI com branco (255)
        cv2.fillPoly(mask, vertices, 255)
        
        return mask

    def normalize_hough_results(self, angle, intersection_x, img_width):
        """
        Normaliza o ângulo e o offset da interseção para o intervalo [-1.0, 1.0].
        
        Args:
            angle (float): Ângulo em graus (0-90).
            intersection_x (int): Posição X da interseção da linha central (0-width).
            img_width (int): Largura do frame.
            
        Returns:
            tuple: (norm_angle, norm_offset_x)
        """
        # Normalização do Ângulo (0 a 90 graus)
        # Transforma de (0 a 90) para (0.0 a 1.0)
        # O ângulo 0 (linha horizontal) se torna 1.0 (máximo desvio)
        # O ângulo 90 (linha vertical) se torna 0.0 (alinhado com o eixo Y)
        
        # Usamos 90.0 - angle, para que 90 deg = 0.0 e 0 deg = 1.0
        norm_angle = (90.0 - angle) / 90.0 
        norm_angle = np.clip(norm_angle, 0.0, 1.0)

        # Normalização do Offset da Interseção
        # Transforma a posição X de (0 a width) para (-1.0 a 1.0)
        center_x = img_width / 2.0
        offset_pixels = intersection_x - center_x # Negativo = Esquerda, Positivo = Direita
        
        # Normaliza o offset pela metade da largura da imagem (metade = offset máximo)
        # Se offset_pixels é 'width/2', o resultado é 1.0
        norm_offset_x = offset_pixels / center_x
        norm_offset_x = np.clip(norm_offset_x, -1.0, 1.0)

        return norm_angle, norm_offset_x

    def hough_line_detection(self, frame):
        # O frame recebido aqui é a imagem binária/escala de cinza de 8 bits
        h, w = [self.height,self.width]
        
        # --- 1. PRÉ-PROCESSAMENTO (Simplificado, mantendo Canny e fechar) ---
        
        # O seu 'frame' já vem de 'binary_threshold' no 'process_frame',
        # mas a aplicação direta de blur e Canny sobre binária pode ser menos ideal
        # do que usar a imagem original. 
        # Mantendo sua lógica de blur/Canny:
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        m = np.median(blur)
        lower = int(max(0, 0.66 * m))
        upper = int(min(255, 1.33 * m))
        edges = cv2.Canny(blur, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # --- 2. APLICAÇÃO DO ROI ---
        # Cria a máscara e aplica ao mapa de arestas
        roi_mask = self.get_roi_mask(edges.shape, SRC_PC)
        edges_roi = cv2.bitwise_and(edges, edges, mask=roi_mask) 
        # 

        # --- 3. DETECÇÃO DE LINHAS (Usando edges_roi) ---
        lines = cv2.HoughLinesP(edges_roi, 1, np.pi/180, 120, minLineLength=80, maxLineGap=15)

        center_x = w // 2
        center_line = (center_x, 0, center_x, h)
        v_center = np.array([0, h], dtype=float)

        min_angle = 90.0
        min_angle_line = None
        closest_inter = None
        max_inter_y = -1
        
        # Busca pela Linha de Menor Ângulo e Interseção (Lógica original)
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                inter = self.line_inter(center_line, (x1, y1, x2, y2))
                
                if inter is not None:
                    ix, iy = inter
                    # Verifica se a interseção está DENTRO do frame
                    if 0 <= ix < w and 0 <= iy < h:
                        v_line = np.array([x2 - x1, y2 - y1], dtype=float)
                        dot = np.dot(v_center, v_line)
                        norms = np.linalg.norm(v_center) * np.linalg.norm(v_line)

                        if norms > 1e-6:
                            cos_theta = np.clip(dot / norms, -1.0, 1.0)
                            angle = float(np.degrees(np.arccos(cos_theta)))
                        else:
                            angle = 90.0

                        if angle < min_angle or (angle == min_angle and iy > max_inter_y):
                            min_angle = angle
                            min_angle_line = (x1, y1, x2, y2)
                            closest_inter = (ix, iy)
                            max_inter_y = iy

        # Cálculo das Áreas de Deslocamento (Lógica original)
        step = 10
        area_left = 0
        area_right = 0
        a = 0
        if lines is not None:
            for y in range(0, h, step):
                delta_xs = []
                for x1, y1, x2, y2 in lines[:, 0]:
                    # Cálculo da posição x na linha y
                    if x2 != x1:
                        m = (y2 - y1) / (x2 - x1)
                        b = y1 - m * x1
                        if abs(m) > 1e-6:
                            x_at_y = int((y - b) / m)
                        else:
                            x_at_y = x1
                    else:
                        x_at_y = x1

                    if 0 <= x_at_y < w:
                        delta_xs.append(x_at_y - center_x)

                if delta_xs:
                    min_dx = min(delta_xs, key=lambda dx: abs(dx))
                    area = abs(min_dx) * step
                    if min_dx < 0:
                        area_left += area
                    else:
                        area_right += area
        
        # --- 4. NORMALIZAÇÃO DOS RESULTADOS CHAVE ---
        norm_angle_output = 0.0
        norm_offset_x_output = 0.0

        if closest_inter is not None:
            ix, iy = closest_inter
            # Normaliza o ângulo mínimo e o offset horizontal da interseção
            norm_angle_output, norm_offset_x_output = self.normalize_hough_results(min_angle, ix, w)
            
        # Retorna os resultados brutos (para debug) e os normalizados
           
            if h and closest_inter[1] != None: a = (h - closest_inter[1])
        return a, lines, min_angle_line, closest_inter, min_angle, area_left, area_right, norm_angle_output, norm_offset_x_output

if __name__ == '__main__':
    try:
        rospy.init_node('vision', anonymous=True)
        rv = RobotVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

   
