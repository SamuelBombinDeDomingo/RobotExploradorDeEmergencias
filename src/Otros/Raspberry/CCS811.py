import time
import busio
import paho.mqtt.client as mqtt
from board import SCL, SDA
from adafruit_ccs811 import CCS811

# Callback cuando se conecta el cliente MQTT
def on_connect(client, userdata, flags, rc):
    pass

# Configuración del MQTT
mqtt_broker = "192.168.8.20"
mqtt_client = mqtt.Client(client_id="CCS811_Sensor", protocol=mqtt.MQTTv311)

# Establecer el callback de conexión
mqtt_client.on_connect = on_connect

# Conectar al broker MQTT
mqtt_client.connect(mqtt_broker)

# Inicializa el bus I2C
i2c = busio.I2C(SCL, SDA)
ccs811 = CCS811(i2c)

while True:
    mqtt_client.loop()  # Mantener la conexión activa

    if ccs811.data_ready:
        eco2 = ccs811.eco2
        tvoc = ccs811.tvoc
        
        # Publicar los valores en MQTT
        mqtt_client.publish("eCO2", eco2)
        mqtt_client.publish("TVOC", tvoc)
            
    time.sleep(1)
