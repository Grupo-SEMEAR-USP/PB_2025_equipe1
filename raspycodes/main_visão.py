

import cv2
import Imgpross  

def run_lane_detection():

    cap = cv2.VideoCapture(0) 

    if not cap.isOpened():
        print("Erro fatal: Não foi possível abrir nenhuma câmera. Verifique as conexões.")
        return

    while True:
        ret, frame_original = cap.read()
        
        if not ret:
            print("Erro: Não foi possível ler o frame da câmera.")
            break

        try:
            dados_processados, img_marcada = Imgpross.processar_frame(frame_original)
            
            dist_centro = dados_processados.get('distancia_centro', 'N/A')
            raio_curva = dados_processados.get('curvatura_raio', 'N/A')
            direcao = dados_processados.get('Direca o', 'N/A')


            print(f"Distância do Centro: {dist_centro} | Raio de Curvatura: {raio_curva} | Direção: {direcao}   ", end="\r")

            cv2.imshow("Detecção de Faixa - Processado", img_marcada)

        except Exception as e:
            print(f"Erro no processamento: {e}")
            cv2.imshow("Detecção de Faixa - Erro", frame_original)


        #Saída (pressionar 'q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nEncerrando o programa...")
            break


    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_lane_detection()
