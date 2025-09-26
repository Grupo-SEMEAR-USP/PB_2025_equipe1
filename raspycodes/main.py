import serial_com 
import threading

#thread de Leitura
def PortReading():
    while True:
        log = serial_com.read_serial()
        if log:
            print(log)

#Thread de Escrita
def PortWriting():
    while True:
        word = input("Digite algo (ou 'exit' para sair): ")
        if word.lower() == "exit":
            break
        serial_com.write_serial(word)


# Criando threads
t1 = threading.Thread(target=PortReading, daemon=True)  # daemon = fecha junto com main
t2 = threading.Thread(target=PortWriting)

# Iniciando
t1.start()
t2.start()

t2.join()  # só espera a thread de escrita, leitura roda em background
