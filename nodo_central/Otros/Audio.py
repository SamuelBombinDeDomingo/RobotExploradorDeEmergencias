import paho.mqtt.client as mqtt
import os
import tempfile
from gtts import gTTS

# Dirección del servidor MQTT
MQTT_BROKER = "192.168.8.20"
MQTT_PORT = 1883
MQTT_TOPIC = "Texto_Salida"

# Directorio donde se guardan o buscan los archivos MP3
MP3_DIRECTORY = "/home/samuel/RobotExploradorDeEmergencias/src/Otros/Sonidos/"

# Asegurar que el directorio existe
os.makedirs(MP3_DIRECTORY, exist_ok=True)

# Callback para cuando se recibe un mensaje
def on_message(client, userdata, msg):
    text = msg.payload.decode().strip()

    if text.endswith(".mp3"):
        mp3_path = os.path.join(MP3_DIRECTORY, text)
        if os.path.exists(mp3_path):
            os.system(f"ffplay -nodisp -autoexit \"{mp3_path}\"")
    else:
        try:
            # Crear y guardar el archivo MP3 con el nombre basado en el texto
            safe_filename = "".join(c for c in text if c.isalnum() or c in (' ', '_', '-')).rstrip()
            mp3_filename = f"{safe_filename}.mp3"
            mp3_path = os.path.join(MP3_DIRECTORY, mp3_filename)
            
            tts = gTTS(text, lang='es')
            tts.save(mp3_path)
            
            os.system(f"ffplay -nodisp -autoexit \"{mp3_path}\"")
        except Exception:
            pass

# Configuración del cliente MQTT
client = mqtt.Client()
client.on_message = on_message

# Conectar al broker y suscribirse
client.connect(MQTT_BROKER, MQTT_PORT)
client.subscribe(MQTT_TOPIC)
client.loop_forever()
