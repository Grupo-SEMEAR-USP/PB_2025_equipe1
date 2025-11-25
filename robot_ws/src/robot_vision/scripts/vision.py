import rospy
import cv2
import numpy as np
import threading

from robot_communication.msg import vision_pattern

# --- PARÂMETROS GLOBAIS (AJUSTAR ESTES VALORES) ---

# Relação pixel-para-Unidade de Distancia Real (CRÍTICO para cálculos do mundo real)
# Ajuste medindo uma seção reta conhecida da estrada
YM_PER_PIX = 30 / 720  # Metros por pixel na dimensão Y (altura)
XM_PER_PIX = 3.7 / 700 # Metros por pixel na dimensão X (largura)

# Parâmetros do Sliding Window(Algoritimo usado para a detecção da rua)
NWINDOWS = 9     # Número de janelas deslizantes
MARGIN = 100     # Largura da janela (+/- MARGIN)
MINPIX = 50      # Número mínimo de pixels para recentralizar a janela


class RobotVision:
    def __init__(self):
        # Inicialização dos parâmetros e subscritores/publicadores ROS
        self.vision_pub = rospy.Publisher('/vision', vision_pattern, queue_size=10)
  
        self.Minv = None
        self.binary_warped_img = None
        self.warped_img = None
        self.cam_img = None
        self.left_fit_pixel = None  # Ajuste em Pixels (para desenho)
        self.right_fit_pixel = None # Ajuste em Pixels (para desenho)
  

        #definindo dados de visão
        self.vision_data = {
            'center_distance': 0.0,
            'curvature_radius': 0.0,
        }

        self.thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.thread.start()

    def camera_loop(self):
        self.cam = cv2.VideoCapture(0, cv2.CAP_V4L2) # Garante backend V4L2

        # Logo após abrir a câmera e antes do loop principal do ROS:
        for i in range(20):
            ret, frame = self.cam.read()
            # Apenas lendo para dar tempo ao sensor se ajustar à luz

        while not rospy.is_shutdown():
            ret, frame = self.cam.read()
            self.process_frame(frame)
            
            self.cam_img = frame
            if not ret:
                rospy.logwarn("Erro: Não foi possível ler o frame da câmera.")
                break
            ## Linha Pra teste visual
            #self.debug_camera()
            
            cv2.waitKey(1)

            # Publicar dados de visão
            vision_msg = vision_pattern()
            vision_msg.offset = self.vision_data['center_distance']
            vision_msg.curvature = self.vision_data['curvature_radius']
            self.vision_pub.publish(vision_msg)

        self.cam.release()

    def process_frame(self, frame):
        # Processamento do frame para detecção de faixas
        img_warped = self.perspective_transform(frame)

        binary_warped = self.binary_threshold(img_warped)


        left_fit, right_fit = self.sliding_window_search(binary_warped)

        self.left_fit = left_fit
        self.right_fit = right_fit  

        curvature_radius = self.calculate_curvature(left_fit, right_fit, binary_warped.shape[0])
        center_distance = self.calculate_center_distance(left_fit, right_fit, binary_warped.shape[1])

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
        
        # Binarização pelo Canal S (bom para cores)
        # <<< ATENÇÃO: AJUSTAR ESTES VALORES >>>
        s_thresh = (160, 255)
        s_binary = cv2.inRange(s_channel, s_thresh[0], s_thresh[1])
        
        # Binarização por Gradiente (bom para faixas brancas)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Gradiente Sobel na direção X
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) 
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
        
        # <<< ATENÇÃO: AJUSTAR ESTES VALORES >>>
        sobel_thresh = (20, 100)
        sobel_binary = cv2.inRange(scaled_sobel, sobel_thresh[0], sobel_thresh[1])

        # Combinar os dois métodos
        # (Ou a faixa é detectada pela cor OU pelo gradiente)
        combined_binary = np.zeros_like(sobel_binary)
        combined_binary[(s_binary == 255) | (sobel_binary == 255)] = 255
        
        return combined_binary
    
    def perspective_transform(self, img):
        # Aplicar transformação de perspectiva
        h, w = img.shape[:2]

        # <<< ATENÇÃO: AJUSTAR ESTES VALORES >>>
        # Pontos de origem (SRC) - O trapézio na imagem original(onde estão as faixas e nossa regiao de interesse)
        # Estes valores dependem dos testes da camera
        src = np.float32([
            (w * 0.15, h * 0.55),  # Top-left
            (w * 0.75, h * 0.55),  # Top-right
            (w * 1, h * 1),  # Bottom-right
            (w * 0, h * 1)   # Bottom-left
        ])

        # Pontos de destino (DST) Onde esse pontos vão ficar na imagem Transformada com visão "de cima"
        dst = np.float32([
            (w * 0.1, 0),        # Top-left
            (w * 0.9, 0),        # Top-right
            (w * 0.9, h),        # Bottom-right
            (w * 0.1, h)         # Bottom-left
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

        left_fit, right_fit = self.average_slope_intercept(binary_warped, left_lane_inds, right_lane_inds)

        return left_fit, right_fit
    
    def average_slope_intercept(self, binary_warped, left_lane_inds, right_lane_inds):
        # Calcular a média dos coeficientes das linhas detectadas
        h,w = binary_warped.shape

        # Encontrar os pixels que pertencem às faixas
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Ajustar uma linha polinomial de 2º grau (quadrática) aos pixels das faixas

        try:
            left_fit = np.polyfit(lefty*YM_PER_PIX, leftx*XM_PER_PIX, 2)
            right_fit = np.polyfit(righty*YM_PER_PIX, rightx*XM_PER_PIX, 2)
        except (np.linalg.LinAlgError,ValueError, TypeError):
            left_fit = np.array([0,0,0])
            right_fit = np.array([0,0,0])

        return left_fit, right_fit
    
    def calculate_curvature(self, left_fit, right_fit, img_height):
        # Calcular o raio de curvatura das faixas
        y_eval = img_height * YM_PER_PIX  # Ponto onde calcular a curvatura (base da imagem)

        # Fórmula do raio de curvatura
        if(left_fit[0] == 0 or right_fit[0] == 0):
            return float('inf')
        left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
        right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

        # Retorna a média dos dois raios
        return (left_curverad + right_curverad) / 2
    
    def calculate_center_distance(self, left_fit, right_fit, img_width):
        # Calcular a distância do centro do robô ao centro da faixa

        try:
            # Posição da faixa esquerda e direita na base da imagem
            left_lane_base = left_fit[0]*(img_width*YM_PER_PIX)**2 + left_fit[1]*(img_width*YM_PER_PIX) + left_fit[2]
            right_lane_base = right_fit[0]*(img_width*YM_PER_PIX)**2 + right_fit[1]*(img_width*YM_PER_PIX) + right_fit[2]

            # Centro da faixa
            lane_center = (left_lane_base + right_lane_base) / 2.0

            # Centro da imagem (posição do robô)
            image_center = (img_width * XM_PER_PIX) / 2.0

            # Distância do centro do robô ao centro da faixa
            center_distance = image_center - lane_center
        except (TypeError, ValueError, np.linalg.LinAlgError):
            center_distance = 0.0
        
        return center_distance

    def debug_camera(self):
        
        frame = self.cam_img
        binary_warped = self.binary_warped_img
        left_fit = self.left_fit
        right_fit = self.right_fit 
        dados = {
            'curvatura_raio': self.vision_data['curvature_radius'],
            'distancia_centro': self.vision_data['center_distance']
        }
        Minv = self.Minv


        #Desenha a área da faixa detectada e os dados na imagem original.

        h, w = frame.shape[:2]
        
    
        ploty = np.linspace(0, h - 1, h)
        
        try:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            #Se os 'fit' falharem (forem [0,0,0])
            left_fitx = ploty * 0
            right_fitx = ploty * 0
            
        # Criar uma imagem para desenhar o polígono da faixa
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Formatar os pontos para cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Desenhar o polígono da faixa (em verde) na imagem "warpada"
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # "Desfazer" o warp da perspectiva
        new_warp = cv2.warpPerspective(color_warp, Minv, (w, h))
        
        # Combinar a imagem original com o polígono da faixa
        img_marcada = cv2.addWeighted(frame, 1, new_warp, 0.3, 0)

        # Adicionar o texto (dados) na imagem
        curvatura = dados.get('curvatura_raio', 0.0)
        offset = dados.get('distancia_centro', 0.0)
        
        cv2.putText(img_marcada, f'Raio de Curvatura: {curvatura:8.0f}', 
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        offset_dir = "Esquerda" if offset > 0 else "Direita"
        if offset == 0: offset_dir = "Centro"
            
        cv2.putText(img_marcada, f'Offset do Centro: {abs(offset):.2f} ({offset_dir})', 
                    (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("Detecção de Faixa - Debug", img_marcada)
        cv2.imshow("Imagem Warpada Binária", binary_warped)
        cv2.imshow("Imagem Warpada", self.warped_img)
        cv2.imshow("Imagem Original", frame)


if __name__ == '__main__':
    try:
        rospy.init_node('vision', anonymous=True)
        rv = RobotVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

   
