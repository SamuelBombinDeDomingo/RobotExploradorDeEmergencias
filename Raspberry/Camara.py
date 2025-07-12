#!/usr/bin/env python3

import cv2
import depthai as dai
import paho.mqtt.client as mqtt
import base64
import signal
import sys

fps = 30
res = dai.MonoCameraProperties.SensorResolution.THE_400_P
poolSize = 24  # default 3, increased to prevent desync

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoL = pipeline.create(dai.node.MonoCamera)
monoL.setCamera("left")
monoL.setResolution(res)
monoL.setFps(fps)
monoL.setNumFramesPool(poolSize)

xoutFloodL = pipeline.create(dai.node.XLinkOut)
xoutFloodL.setStreamName('flood-left')  # Solo stream de 'flood-left'

# Callback cuando el cliente MQTT se conecta
def on_connect(client, userdata, flags, rc):
    # Eliminar print aquí si no deseas mostrar mensajes
    pass

mqtt_broker = "192.168.8.20"
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Rasberry")
mqtt_client.on_connect = on_connect
mqtt_client.connect(mqtt_broker)
mqtt_client.loop_start()  # Ejecuta el loop en un hilo separado

# Script node for frame routing y alternancia de IR flood
script = pipeline.create(dai.node.Script)
script.setProcessor(dai.ProcessorType.LEON_CSS)
script.setScript("""
    floodBright = 0.1
    while True:
        event = node.io['event'].get()
        Device.setIrFloodLightIntensity(floodBright)

        frameL = node.io['frameL'].get()
        node.io['floodL'].send(frameL)
""")

# Linking
monoL.frameEvent.link(script.inputs['event'])
monoL.out.link(script.inputs['frameL'])
script.outputs['floodL'].link(xoutFloodL.input)

# Manejo de señales para detener correctamente
def signal_handler(sig, frame):
    # Eliminar print aquí si no deseas mostrar mensajes
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    queue = device.getOutputQueue(name='flood-left', maxSize=4, blocking=False)

    while True:
        pkt = queue.tryGet()
        if pkt is not None:
            frame = pkt.getCvFrame()
            # Codificar y publicar la imagen en el canal MQTT
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            mqtt_client.publish("Camara", jpg_as_text)

