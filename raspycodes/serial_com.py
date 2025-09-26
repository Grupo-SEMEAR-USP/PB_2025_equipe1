# Importing Libraries 
import serial as sr
##Define a serial
serial = sr.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

#Lê as informações Recebidas pelo Serial e Retorna elas com String
def read_serial():
    
    try: #inicia Tentativa de leitura 

        if serial.in_waiting > 0:  # se tem algo no buffer(ou seja Se algo foi mandado)

            # Dados recebidos = Leitura decodificada(Bytes=>String) ignorando os erros e formatando
            data = serial.readline().decode(errors="ignore").strip()
            if data: #Se leu alguma coisa
               
                return(f"{data} \n")  #retorna oq leu
            
   
    except Exception as e:  #deu merda
        print("Erro na leitura:", e)

def write_serial(x):
    try:
        serial.write((x + "\n").encode())  # envia como bytes
    except Exception as e:
        print("Erro na escrita:", e)#deu Merda na Escrita