import os
import pyaudio
import json
import paho.mqtt.client as mqtt
from vosk import Model, KaldiRecognizer
import time

# Configuración de MQTT
MQTT_BROKER = "192.168.8.20"
MQTT_PORT = 1883
MQTT_TOPIC_TEXT = "Texto_Entrada"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

# Cargar el modelo de Vosk
model_path = "RobotExploradorDeEmergencias/src/Otros/vosk-model-small-es-0.42"
model = Model(model_path)

# Configurar el micrófono
recognizer = KaldiRecognizer(model, 16000)
p = pyaudio.PyAudio()

# Iniciar el micrófono
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=4000)
stream.start_stream()

# Bucle principal
while True:
    try:
        data = stream.read(4000, exception_on_overflow=False)
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            result_json = json.loads(result)
            client.publish(MQTT_TOPIC_TEXT, result_json['text'])
    except IOError:
        pass  # Se omite el error
    time.sleep(0.1)

