import os
import signal
import subprocess
import time
import threading
import paho.mqtt.client as mqtt

# Nombres de los procesos
node_name = "sllidar_node"
camera_script_name = "Camara.py"

# Dirección del servidor MQTT (Node-RED en este caso)
mqtt_broker = "192.168.8.20"
mqtt_topic_camera = "Camara"
mqtt_topic_lidar = "Lidar"

# Variables para almacenar los procesos lanzados
launch_process = None  # Para LIDAR
camera_process = None  # Para Cámara

# Bandera de estado para cada dispositivo
camera_should_run = False
lidar_should_run = False

# Encuentra el PID del nodo LIDAR en ejecución
def get_pid_of_node(node_name):
    try:
        pid_output = subprocess.check_output(f"pgrep -f {node_name}", shell=True).decode().strip()
        return int(pid_output.split()[0]) if pid_output else None
    except subprocess.CalledProcessError:
        return None

# Envía señal de interrupción a un proceso
def send_interrupt_signal(pid):
    if pid:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass  # Si el proceso ya no existe

# Verifica si la cámara está conectada
def is_camera_connected():
    try:
        result = subprocess.run(
            ["python3", "-c", "import depthai; depthai.Device()"], 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except Exception:
        return False

# Lanza el LIDAR si no está ejecutándose
def launch_ros2():
    global launch_process
    if not launch_process:
        try:
            launch_process = subprocess.Popen(["ros2", "launch", "sllidar_ros2", "sllidar_a2m12_launch.py"])
        except Exception:
            pass

# Detiene el LIDAR
def stop_ros2_launch():
    global launch_process
    if launch_process:
        launch_process.terminate()
        launch_process = None

# Lanza el script de la cámara si no está en ejecución
def launch_camera():
    global camera_process
    if not camera_process and is_camera_connected():
        try:
            camera_process = subprocess.Popen(["python3", camera_script_name])
        except Exception:
            pass

# Detiene la cámara
def stop_camera():
    global camera_process
    if camera_process:
        camera_process.terminate()
        camera_process = None

# Monitorea la cámara y el LIDAR
def monitor_system():
    global camera_should_run, lidar_should_run, camera_process, launch_process

    while True:
        if camera_should_run:
            if not camera_process:
                launch_camera()
        else:
            stop_camera()

        if lidar_should_run:
            if not launch_process:
                launch_ros2()
        else:
            stop_ros2_launch()

        if camera_should_run and camera_process and camera_process.poll() is not None:
            camera_process = None

        if lidar_should_run and launch_process and launch_process.poll() is not None:
            launch_process = None

        time.sleep(0.5)

# Función para verificar si el LIDAR debería estar en ejecución
def check_lidar_guardian():
    global lidar_should_run
    while True:
        if lidar_should_run:
            # Intenta lanzar el LIDAR si no está en ejecución
            if not launch_process:
                print("Intentando lanzar el LIDAR...")
                launch_ros2()

        time.sleep(1)  # Verifica cada 1 segundo si el LIDAR debería ejecutarse

# Función para manejar mensajes MQTT
def on_message(client, userdata, msg):
    global camera_should_run, lidar_should_run

    message = msg.payload.decode().strip()
    topic = msg.topic

    if topic == mqtt_topic_camera:
        if message == '1':
            camera_should_run = True
        elif message == '0':
            camera_should_run = False

    elif topic == mqtt_topic_lidar:
        if message == '1':
            lidar_should_run = True
        elif message == '0':
            lidar_should_run = False
            pid = get_pid_of_node(node_name)
            if pid:
                send_interrupt_signal(pid)

# Configura el cliente MQTT
def setup_mqtt():
    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.on_message = on_message
    client.connect(mqtt_broker)
    client.subscribe([(mqtt_topic_camera, 0), (mqtt_topic_lidar, 0)])
    return client

if __name__ == "__main__":
    mqtt_client = setup_mqtt()

    monitor_thread = threading.Thread(target=monitor_system, daemon=True)
    monitor_thread.start()

    # Hilo del perro guardián para el LIDAR
    lidar_guardian_thread = threading.Thread(target=check_lidar_guardian, daemon=True)
    lidar_guardian_thread.start()

    mqtt_client.loop_forever()
