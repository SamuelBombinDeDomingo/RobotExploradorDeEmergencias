import time
import paho.mqtt.client as mqtt
import subprocess

# Callback cuando se conecta el cliente MQTT
def on_connect(client, userdata, flags, rc):
    pass

# Configuración del MQTT
mqtt_broker = "192.168.8.20"
mqtt_client = mqtt.Client(client_id="RSSI_Sensor", protocol=mqtt.MQTTv311)

# Establecer el callback de conexión
mqtt_client.on_connect = on_connect

# Conectar al broker MQTT
mqtt_client.connect(mqtt_broker)

def get_rssi():
    # Ejecuta el comando para obtener el RSSI
    result = subprocess.run(['iwconfig'], capture_output=True, text=True)
    for line in result.stdout.splitlines():
        if 'Signal level' in line:
            # Extrae el valor del RSSI
            rssi = line.split('Signal level=')[1].split(' ')[0]
            return int(rssi)
    return None

while True:
    mqtt_client.loop()  # Mantener la conexión activa

    rssi = get_rssi()
    if rssi is not None:
        # Publicar el valor de RSSI en MQTT
        mqtt_client.publish("RSSI", rssi)

    time.sleep(1)
