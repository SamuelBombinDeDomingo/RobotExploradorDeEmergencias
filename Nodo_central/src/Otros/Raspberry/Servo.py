import paho.mqtt.client as mqtt
from gpiozero import Servo
from time import sleep

# Configuración del servomotor en el pin GPIO 18
servo = Servo(18)

# Dirección del servidor MQTT
mqtt_broker = "192.168.8.20"  # IP de tu broker MQTT
mqtt_port = 1883  # Puerto estándar MQTT
mqtt_topic = "Servo"  # Tópico donde recibiremos los comandos

# Función que maneja los mensajes MQTT recibidos
def on_message(client, userdata, msg):
    # Decodificar el mensaje recibido
    try:
        value = float(msg.payload.decode('utf-8'))  # Convertir el mensaje a un valor numérico
        if -1 <= value <= 1:
            # Mover el servomotor al valor recibido (-1 a 1)
            servo.value = value  # Controla el servomotor
    except ValueError:
        pass  # Si no es un número válido, no hacer nada

# Crear el cliente MQTT y configurar las funciones de conexión y mensajes
client = mqtt.Client()

# Conectar al broker MQTT
client.connect(mqtt_broker, mqtt_port, 60)

# Suscribirse al tópico "Servo"
client.subscribe(mqtt_topic)

# Asignar la función que maneja los mensajes
client.on_message = on_message

# Empezar a escuchar los mensajes en segundo plano
client.loop_start()

# Mantener el script ejecutándose para recibir mensajes
try:
    while True:
        sleep(1)  # El programa espera, pero sigue recibiendo mensajes
except KeyboardInterrupt:
    client.loop_stop()  # Detener el loop MQTT al finalizar el programa
