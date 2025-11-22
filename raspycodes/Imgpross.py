# Imgpross.py

import cv2
import numpy as np

# --- PARÂMETROS GLOBAIS (AJUSTAR ESTES VALORES) ---

# Relação pixel-para-Unidade de Distancia Real (CRÍTICO para cálculos do mundo real)
# Ajuste medindo uma seção reta conhecida da estrada
YM_PER_PIX = 30 / 720  # Metros por pixel na dimensão Y (altura)
XM_PER_PIX = 3.7 / 700 # Metros por pixel na dimensão X (largura)

# Parâmetros do Sliding Window(Algoritimo usado para a detecção da rua)
NWINDOWS = 9     # Número de janelas deslizantes
MARGIN = 100     # Largura da janela (+/- MARGIN)
MINPIX = 50      # Número mínimo de pixels para recentralizar a janela

# --- Variáveis Globais para Suavização (WIP Sugestão do GPT kkkk) ---
# (Pode ser implementado depois para reduzir o "tremor" dos valores)
# left_fit_hist = []
# right_fit_hist = []


def perspectiva_warp(img, img_size):
    
    # Aplica a transformação de perspectiva (visão de cima/bird's-eye view).
    
    # Retorna:
    #- M: A matriz de transformação(Literalmente uma matriz que faz a transformaçao kkkkk)
    #- Minv: A matriz de transformação inversa (A matriz Pra desfazer a transformaçao kkkkk)
    #- warped_img: A imagem transformada (a imagem "de cima")
    
    h, w = img_size 
    
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
    Minv = cv2.getPerspectiveTransform(dst, src)
    
    #Aplicar o warp/transformação
    warped_img = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)
    
    return M, Minv, warped_img


def binarizar_imagem(img):

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


def encontrar_faixas_sliding_windows(binary_warped):
   
    #Encontra os pixels das faixas usando a técnica de sliding windows.
    
    #Retorna:
    #- left_fit: Coeficientes da parábola da faixa esquerda
    #- right_fit: Coeficientes da parábola da faixa direita
    #- left_lane_inds: Índices dos pixels da faixa esquerda
    #- right_lane_inds: Índices dos pixels da faixa direita
    #- out_img: Imagem de visualização das janelas (para debug)
 
    
    h, w = binary_warped.shape

    #Cria histograma da metade inferior da imagem
    histogram = np.sum(binary_warped[h // 2:, :], axis=0)
    
    # Imagem de saída para visualização
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    # Encontrar os picos do histograma (base das faixas)
    midpoint = np.int32(w / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    #Configurar as janelas
    win_height = np.int32(h / NWINDOWS)
    
    # Identificar posições de pixels != 0
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    leftx_current = leftx_base
    rightx_current = rightx_base

    left_lane_inds = []
    right_lane_inds = []

    # Itera sobre as janelas(GPT QUE FEZ KKKK)
    for window in range(NWINDOWS):
        # Definir limites da janela
        win_y_low = h - (window + 1) * win_height
        win_y_high = h - window * win_height
        win_xleft_low = leftx_current - MARGIN
        win_xleft_high = leftx_current + MARGIN
        win_xright_low = rightx_current - MARGIN
        win_xright_high = rightx_current + MARGIN

        # Desenhar as janelas (para visualização)
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

        # Identificar pixels não-zero dentro da janela
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        # Adicionar estes índices à lista
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        #encontramos pixels > minpix, recentralizar a próxima janela
        if len(good_left_inds) > MINPIX:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > MINPIX:
            rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

    # Concatenar os índices
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Se uma faixa não for encontrada, passa adiante
        pass

    # Extrair coordenadas dos pixels
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Interpola uma parábola (polinômio de 2º grau)
    #    (com try/except para caso uma faixa não seja detectada)
    try:
        left_fit = np.polyfit(lefty, leftx, 2)
    except (np.linalg.LinAlgError, TypeError, ValueError):
        left_fit = [0, 0, 0] # Falha no ajuste
        
    try:
        right_fit = np.polyfit(righty, rightx, 2)
    except (np.linalg.LinAlgError, TypeError, ValueError):
        right_fit = [0, 0, 0] # Falha no ajuste
        
    return left_fit, right_fit, left_lane_inds, right_lane_inds, out_img


def calcular_medidas(binary_warped, left_lane_inds, right_lane_inds):
 
    #Calcula o raio de curvatura e o deslocamento do centro.
  
    h, w = binary_warped.shape
    
    # Posições dos pixels (vem da função anterior)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    curvatura_raio = 1e6  # Valor padrão (muito grande/reto)
    distancia_centro = 0.0   # Valor padrão (centralizado)

    # Ajusta novos polinômios usando as Conversões para unidade citadas la em cima (escalas XM/YM_PER_PIX)
    try:
        left_fit_cr = np.polyfit(lefty * YM_PER_PIX, leftx * XM_PER_PIX, 2)
        right_fit_cr = np.polyfit(righty * YM_PER_PIX, rightx * XM_PER_PIX, 2)
    except (np.linalg.LinAlgError, TypeError, ValueError):
        # Falha em encontrar faixas, retorna os padrões
        return curvatura_raio, distancia_centro

    #    Calcular o Raio de Curvatura
    #    Avalia no ponto mais baixo da imagem (mais próximo do Robo)
    y_eval = h * YM_PER_PIX
    
    # Fórmula do raio de curvatura: R = (1 + (2Ay + B)^2)^(3/2) / |2A| (gpt que fez dnv kkkkk)
    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])
    
    # Usa a média das duas faixas
    curvatura_raio = (left_curverad + right_curverad) / 2

    #    Calcular Deslocamento do Centro
    #    Calcula o centro da faixa na base da imagem (y = h)
    
    try:
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        left_x_bottom = left_fit[0] * h**2 + left_fit[1] * h + left_fit[2]
        right_x_bottom = right_fit[0] * h**2 + right_fit[1] * h + right_fit[2]
        
        # Centro da faixa (em pixels)
        lane_center_pixels = (left_x_bottom + right_x_bottom) / 2
        
        # Centro do carro (assumindo que a câmera está no centro)
        car_center_pixels = w / 2
        
        # Deslocamento
        offset_pixels = car_center_pixels - lane_center_pixels
        distancia_centro = offset_pixels * XM_PER_PIX
        
    except (np.linalg.LinAlgError, TypeError, ValueError):
        # Falha em encontrar faixas, retorna os padrões
        pass

    return curvatura_raio, distancia_centro


def desenhar_visualizacao(img_original, binary_warped, Minv, left_fit, right_fit, dados):
  
    #Desenha a área da faixa detectada e os dados na imagem original.

    h, w = img_original.shape[:2]
    
   
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
    img_marcada = cv2.addWeighted(img_original, 1, new_warp, 0.3, 0)

    # Adicionar o texto (dados) na imagem
    curvatura = dados.get('curvatura_raio', 0.0)
    offset = dados.get('distancia_centro', 0.0)
    
    cv2.putText(img_marcada, f'Raio de Curvatura: {curvatura:8.0f}', 
                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    offset_dir = "Esquerda" if offset > 0 else "Direita"
    if offset == 0: offset_dir = "Centro"
        
    cv2.putText(img_marcada, f'Offset do Centro: {abs(offset):.2f} ({offset_dir})', 
                (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    return img_marcada



def processar_frame(frame):
    
    img_size = (frame.shape[0], frame.shape[1])
    
    M, Minv, img_warped = perspectiva_warp(frame, img_size)

    # Binariza a imagem "warpada"
    img_binarizada = binarizar_imagem(img_warped)
    
    #    (Retorna coeficientes em PIXELS)
    left_fit, right_fit, left_inds, right_inds, out_img_sw = encontrar_faixas_sliding_windows(img_binarizada)

    #  Calcula curvatura e offset
    #  (Usa os índices dos pixels e as constantes de conversão)
    curvatura, offset = calcular_medidas(img_binarizada, left_inds, right_inds)

    # 5. Criar o dicionário de dados
    dados_processados = {
        "distancia_centro": offset,
        "curvatura_raio": curvatura,
        "Direcao": "Esquerda" if offset > 0 else "Direita" if offset < 0 else "Centro"
    }


    img_marcada = desenhar_visualizacao(frame, img_binarizada, Minv, left_fit, right_fit, dados_processados)

    # (Para Debug) Se quiser ver as janelas, pode combinar as imagens:
    # out_img_sw_resized = cv2.resize(out_img_sw, (w//4, h//4))
    
    # Retorna os dados e a imagem final
    return dados_processados, img_marcada