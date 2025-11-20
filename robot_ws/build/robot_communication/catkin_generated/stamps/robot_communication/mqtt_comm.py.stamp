#!/usr/bin/env python3
import rospy
import json
import math
from paho.mqtt import client as mqtt
import threading

from robot_communication.msg import encoder_comm, velocity_comm

BROKER = "localhost"
PORT = 1883

TOPIC_ENCODER = "comm/encoders"
TOPIC_COMMAND = "comm/motors"

class MQTTcomm:
    def __init__(self): 
        # MQTT
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # ROS pubs/subs existentes
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_comm, queue_size=10)
        self.sub_velocity = rospy.Subscriber('/velocity_cmd', velocity_comm, self.cb_vel)

        # Conexão broker
        self.client.connect(BROKER, PORT, 60)

        # Definições Encoder
        self.encoder_data = {
            "left": 0.0,
            "right": 0.0
        }
        self.encoder_lock = threading.Lock()

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo("Conectado ao broker MQTT (rc=%s)", rc)
        client.subscribe([(TOPIC_ENCODER, 0)])

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == TOPIC_ENCODER:
                with self.encoder_lock:
                    if "left" in payload:
                        self.encoder_data["left"] = float(payload["left"])
                    if "right" in payload:
                        self.encoder_data["right"] = float(payload["right"])
                self.publish_encoders()

        except Exception as e:
            rospy.logerr(f"Erro ao processar mensagem MQTT: {e}")

    def publish_encoders(self):
        encoder_msg = encoder_comm()

        with self.encoder_lock:
            encoder_msg.left_enc  = self.encoder_data["left"]
            encoder_msg.right_enc = self.encoder_data["right"]

        self.pub_encoder.publish(encoder_msg)

    def cb_vel(self, msg):
        try:
            data = {
                "left":  msg.left_vel,
                "right": msg.right_vel
            }

            self.client.publish(TOPIC_COMMAND, json.dumps(data), qos=0)

        except Exception as e:
            rospy.logerr(f"Erro ao enviar comandos MQTT: {e}")

    def loop(self):
        self.client.loop_forever()

if __name__ == "__main__":
    rospy.init_node('mqtt_comm')
    bridge = MQTTcomm()
    rospy.spin()