#!/usr/bin/env python3
import rospy
import serial
import struct
import threading
from robot_communication.msg import encoder_comm, velocity_comm

# Configurações da Serial
# Verifique se é /dev/ttyUSB0 ou /dev/ttyACM0 com 'ls /dev/tty*'
SERIAL_PORT = "/dev/ttyUSB0" 
BAUD_RATE = 115200

# Definições do Protocolo (Iguais ao ESP32)
FRAME_SOF = 0xAA
FRAME_EOF = 0xBB
PAYLOAD_SIZE = 8 # 2 floats de 4 bytes
FRAME_SIZE = 1 + PAYLOAD_SIZE + 1 + 1 # SOF + Payload + CHK + EOF = 11 bytes

class UARTComm:
    def __init__(self):
        rospy.init_node('uart_comm', anonymous=False)
        
        # Publishers e Subscribers
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_comm, queue_size=10)
        self.sub_velocity = rospy.Subscriber('/velocity_cmd', velocity_comm, self.cb_vel)

        # Inicializa a porta Serial
        try:
            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.1 
            )
            rospy.loginfo(f"Conectado à porta serial: {SERIAL_PORT} a {BAUD_RATE} bps")
        except serial.SerialException as e:
            rospy.logerr(f"Falha ao abrir porta serial: {e}")
            exit(1)

        # Controle de Thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def calculate_checksum(self, data_bytes):
        """
        Realiza a soma simples dos bytes e retorna o byte menos significativo
        """
        return sum(data_bytes) & 0xFF

    def cb_vel(self, msg):
        """
        Recebe comando de velocidade do ROS e envia para o ESP32
        """
        try:
            # Empacota os dois floats (Little Endian '<', 2 floats 'ff')
            payload = struct.pack('<ff', msg.left_vel, msg.right_vel)
            
            # Calcula Checksum
            checksum = self.calculate_checksum(payload)
            
            # Monta o Frame Completo
            frame = struct.pack('B', FRAME_SOF) + payload + struct.pack('B', checksum) + struct.pack('B', FRAME_EOF)
            
            # Envia via Serial
            self.serial_port.write(frame)
            
        except Exception as e:
            rospy.logerr(f"Erro ao enviar comando UART: {e}")

    def read_loop(self):
        """
        Loop contínuo para ler dados vindos do ESP32
        """
        expected_sof = struct.pack('B', FRAME_SOF)
        expected_eof = struct.pack('B', FRAME_EOF)

        while self.running and not rospy.is_shutdown():
            try:
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    
                    if byte == expected_sof:
                        rest_of_frame = self.serial_port.read(FRAME_SIZE - 1)
                        
                        if len(rest_of_frame) == (FRAME_SIZE - 1):
                            payload = rest_of_frame[0:8]
                            received_chk = rest_of_frame[8:9]
                            received_eof = rest_of_frame[9:10]
                            
                            # Validação EOF
                            if received_eof != expected_eof:
                                rospy.logwarn("Erro de EOF na serial")
                                continue

                            # Validação Checksum
                            calculated_chk_int = self.calculate_checksum(payload)
                            calculated_chk_byte = struct.pack('B', calculated_chk_int)

                            if received_chk != calculated_chk_byte:
                                rospy.logwarn("Erro de Checksum na serial")
                                continue

                            # Desempacota os dados
                            left_enc, right_enc = struct.unpack('<ff', payload)
                            self.publish_encoders(left_enc, right_enc)
                            
            except Exception as e:
                rospy.logerr(f"Erro na leitura serial: {e}")
                rospy.sleep(1)

    def publish_encoders(self, left, right):
        msg = encoder_comm()
        msg.left_enc = left
        msg.right_enc = right
        self.pub_encoder.publish(msg)

    def shutdown(self):
        self.running = False
        if self.serial_port.is_open:
            self.serial_port.close()

if __name__ == "__main__":
    uart_node = UARTComm()
    rospy.on_shutdown(uart_node.shutdown)
    rospy.spin()