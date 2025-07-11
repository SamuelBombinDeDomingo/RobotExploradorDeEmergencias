
import os
import paho.mqtt.client as mqtt
import subprocess
import time

MQTT_BROKER = "192.168.8.20"
MQTT_PORT = 1883
MQTT_TOPIC_TEXT = "Texto_Salida"

CARPETA_SONIDOS = "/home/samuel/RobotExploradorDeEmergencias/src/Otros/Sonidos"

import os
import paho.mqtt.client as mqtt
import subprocess
import time

MQTT_BROKER = "192.168.8.20"
MQTT_PORT = 1883
MQTT_TOPIC_TEXT = "Texto_Salida"

CARPETA_SONIDOS = "/home/samuel/RobotExploradorDeEmergencias/src/Otros/Sonidos"

# Obtener lista de archivos .mp3 disponibles
archivos_disponibles = [f for f in os.listdir(CARPETA_SONIDOS) if f.endswith(".mp3")]

def reproducir_sonido(nombre_mp3):
    ruta = os.path.join(CARPETA_SONIDOS, nombre_mp3)
    subprocess.run(["ffplay", "-nodisp", "-autoexit", ruta],
                   stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_TOPIC_TEXT)

def on_message(client, userdata, msg):
    mensaje = msg.payload.decode().strip()
    mensaje_sin_espacios = mensaje.replace(" ", "").lower()
    archivo_encontrado = None
    for archivo in archivos_disponibles:
        archivo_sin_ext = os.path.splitext(archivo)[0]
        archivo_formateado = archivo_sin_ext.replace(" ", "").lower()
        if archivo_formateado == mensaje_sin_espacios:
            archivo_encontrado = archivo
            break
    if archivo_encontrado:
        reproducir_sonido(archivo_encontrado)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

try:
    while True:
        time.sleep(0.03)
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()

