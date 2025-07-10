import base64
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
import os
import time

# Configuración del broker MQTT y los tópicos
MQTT_BROKER = "192.168.8.20"
MQTT_RECEIVE = "Camara"
MQTT_CONTROL = "Casa"

# Configuración de la ventana
window_x = 0  # Posición x en la esquina superior izquierda
window_y = 0  # Posición y en la esquina superior izquierda
window_width = 645  # Ancho de la ventana
window_height = 480  # Alto de la ventana

# Configuración del video
fps = 20.0  # Frames por segundo
frame_time = 1 / fps

# Inicializar variables
frame = np.zeros((window_height, window_width), np.uint8)
mostrar_ventana = False
grabando_video = False
mensaje_control = "0"
video_writer = None

# Callback al conectar con el broker
def on_connect(client, userdata, flags, reasonCode, properties=None):
    client.subscribe(MQTT_RECEIVE)
    client.subscribe(MQTT_CONTROL)

# Callback al recibir mensajes
def on_message(client, userdata, msg):
    global frame, mensaje_control

    if msg.topic == MQTT_RECEIVE:
        try:
            img = base64.b64decode(msg.payload)
            npimg = np.frombuffer(img, dtype=np.uint8)
            frame = cv.imdecode(npimg, cv.IMREAD_GRAYSCALE)
        except:
            pass

    elif msg.topic == MQTT_CONTROL:
        mensaje_control = msg.payload.decode().strip()

# Configurar MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, port=1883, keepalive=60)
client.loop_start()

try:
    while True:
        start_time = time.time()

        if mensaje_control == "1":
            if not mostrar_ventana:
                cv.namedWindow("Cámara", cv.WINDOW_GUI_NORMAL)
                cv.resizeWindow("Cámara", window_width, window_height)
                cv.moveWindow("Cámara", window_x, window_y)
                mostrar_ventana = True

            if not grabando_video:
                fourcc = cv.VideoWriter_fourcc(*'XVID')
                video_filename = os.path.join("RobotExploradorDeEmergencias/src/Otros", "video.avi")
                video_writer = cv.VideoWriter(video_filename, fourcc, fps, (window_width, window_height), isColor=False)
                grabando_video = True

            if frame is not None and frame.size > 0:
                frame_resized = cv.resize(frame, (window_width, window_height), interpolation=cv.INTER_LINEAR)
                cv.imshow("Cámara", frame_resized)
                cv.moveWindow("Cámara", window_x, window_y)

                frame_to_save = frame_resized.copy()
                current_time = time.strftime("%Y-%m-%d %H:%M:%S")
                font = cv.FONT_HERSHEY_SIMPLEX
                cv.putText(frame_to_save, current_time, (10, 30), font, 1, (255, 255, 255), 2, cv.LINE_AA)

                if video_writer is not None and video_writer.isOpened():
                    video_writer.write(frame_to_save)

            elapsed_time = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed_time)
            time.sleep(sleep_time)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            if grabando_video:
                if video_writer is not None and video_writer.isOpened():
                    video_writer.release()
                grabando_video = False

            if mostrar_ventana:
                cv.destroyWindow("Cámara")
                mostrar_ventana = False

finally:
    if video_writer is not None and grabando_video:
        video_writer.release()
    cv.destroyAllWindows()
    client.loop_stop()
    client.disconnect()

