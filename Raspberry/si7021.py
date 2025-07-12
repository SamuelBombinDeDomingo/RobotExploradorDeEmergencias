import time
import busio
import paho.mqtt.client as mqtt
from board import SCL, SDA
from adafruit_si7021 import SI7021

# Configuración del MQTT
mqtt_broker = "192.168.8.20"  # Dirección del broker MQTT
mqtt_client = mqtt.Client(client_id="SI7021_Sensor", protocol=mqtt.MQTTv311)

# Establecer la función callback cuando se conecta al broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        pass  # Se conecta exitosamente, no es necesario hacer nada aquí
    else:
        pass  # Error al conectar con el broker

# Conectar al broker MQTT
mqtt_client.on_connect = on_connect
mqtt_client.connect(mqtt_broker)

# Inicializa el bus I2C
i2c = busio.I2C(SCL, SDA)
sensor = SI7021(i2c)  # Crear una instancia del sensor

# Mantener la conexión MQTT activa
mqtt_client.loop_start()  # Esto ejecuta un hilo en segundo plano para manejar las comunicaciones MQTT

while True:
    # Leer la temperatura y la humedad
    temperature = round(sensor.temperature, 2)  # Redondear a 2 decimales
    humidity = round(sensor.relative_humidity, 2)  # Redondear a 2 decimales

    # Publicar los valores por MQTT
    mqtt_client.publish("Temperatura", temperature)
    mqtt_client.publish("Humedad", humidity)

    time.sleep(1)  # Esperar 1 segundo antes de tomar la siguiente lectura

