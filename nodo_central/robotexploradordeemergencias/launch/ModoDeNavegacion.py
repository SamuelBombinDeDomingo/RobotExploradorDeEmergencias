import paho.mqtt.client as mqtt
import subprocess
import signal
import os
import pathlib

# Rutas base absolutas
BASE_PATH = '/home/samuel/Escritorio/RobotExploradorDeEmergencias/Emplazamiento'

def kill_processes():
    global processes, balizas_process
    for p in processes:
        if p and p.poll() is None:  # Está vivo
            try:
                sig = signal.SIGINT if balizas_process and p.pid == balizas_process.pid else signal.SIGTERM
                os.killpg(os.getpgid(p.pid), sig)
                p.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
    processes.clear()
    balizas_process = None

def kill_specific_process(proc, use_sigint=False):
    if proc and proc.poll() is None:
        try:
            sig = signal.SIGINT if use_sigint else signal.SIGTERM
            os.killpg(os.getpgid(proc.pid), sig)
            proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass

def on_message(client, userdata, msg):
    global processes, last_mode, script_process, balizas_process
    topic = msg.topic
    payload = msg.payload.decode().strip()

    if topic == MQTT_TOPIC_MODO:
        kill_processes()
        if script_process is not None:
            kill_specific_process(script_process)
            script_process = None

        last_mode = payload
        if payload == '1':
            processes.append(subprocess.Popen(RVIZ_LAUNCH, shell=True, preexec_fn=os.setsid))
        elif payload == '2':
            processes.append(subprocess.Popen(RVIZ_LAUNCH_1, shell=True, preexec_fn=os.setsid))
            processes.append(subprocess.Popen(SLAM_LAUNCH, shell=True, preexec_fn=os.setsid))
        elif payload == '3':
            processes.append(subprocess.Popen(RVIZ_LAUNCH_2, shell=True, preexec_fn=os.setsid))
            processes.append(subprocess.Popen(NAV2_LAUNCH, shell=True, preexec_fn=os.setsid))

    elif topic == MQTT_TOPIC_ARCHIVO:
        directory = os.path.join(BASE_PATH, payload)

        if last_mode != '3':
            if script_process is not None:
                kill_specific_process(script_process)
                script_process = None

        if last_mode == '3':
            if script_process is not None:
                kill_specific_process(script_process)
                script_process = None

            script_path = os.path.join(BASE_PATH, payload, 'Posiciones.py')
            if os.path.isfile(script_path):
                script_process = subprocess.Popen(['python3', script_path], preexec_fn=os.setsid)

            localization_command = LOCALIZATION_LAUNCH_TEMPLATE.format(payload, payload)
            posicion_script = POSICION_SCRIPT_TEMPLATE.format(payload)
            balizas_script = f'python3 {os.path.join(BASE_PATH, payload, "Balizas.py")}'

            processes.append(subprocess.Popen(localization_command, shell=True, preexec_fn=os.setsid))
            processes.append(subprocess.Popen(posicion_script, shell=True, preexec_fn=os.setsid))
            balizas_process = subprocess.Popen(balizas_script, shell=True, preexec_fn=os.setsid)
            processes.append(balizas_process)

        elif last_mode == '2':
            pathlib.Path(directory).mkdir(parents=True, exist_ok=True)
            save_map_command = (
                'ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '
                '"{name: {data: \'' + directory + '/' + payload + '\'}}"'
            )
            subprocess.run(save_map_command, shell=True, check=False)

# Configuración MQTT
MQTT_BROKER = '192.168.8.20'
MQTT_PORT = 1883
MQTT_TOPIC_MODO = 'Modo_Navegacion'
MQTT_TOPIC_ARCHIVO = 'Archivo'

# Comandos ROS 2 (usar rutas absolutas si hace falta)
LOCALIZATION_LAUNCH_TEMPLATE = f'ros2 launch navegacion localization.launch.py map:={BASE_PATH}/{{}}/{{}}.yaml'
POSICION_SCRIPT_TEMPLATE = f'python3 {BASE_PATH}/{{}}/Posicion.py'
RVIZ_LAUNCH = 'ros2 launch irobot_create_common_bringup rviz2.launch.py'
RVIZ_LAUNCH_1 = 'ros2 launch irobot_create_common_bringup rviz2_1.launch.py'
RVIZ_LAUNCH_2 = 'ros2 launch irobot_create_common_bringup rviz2_2.launch.py'
NAV2_LAUNCH = 'ros2 launch navegacion nav2.launch.py'
SLAM_LAUNCH = 'ros2 launch navegacion slam.launch.py'

# Variables globales
processes = []
script_process = None
balizas_process = None
last_mode = None

# Cliente MQTT
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.subscribe([(MQTT_TOPIC_MODO, 0), (MQTT_TOPIC_ARCHIVO, 0)])

client.loop_forever()

