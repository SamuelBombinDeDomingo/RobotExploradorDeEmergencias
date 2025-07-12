import os
import signal
import subprocess
import time
import threading
import paho.mqtt.client as mqtt

node_name = "sllidar_node"
camera_script_name = "Camara.py"

mqtt_broker = "192.168.8.20"
mqtt_topic_lidar = "Lidar"

launch_process = None
camera_process = None

lidar_should_run = False

watchdog_retry_delay = 5  # segundos

watchdog_active = False

def is_camera_connected():
    try:
        result = subprocess.run(
            ["python3", "-c", "import depthai; depthai.Device()"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except Exception:
        return False

def launch_ros2():
    global launch_process
    if not launch_process:
        try:
            launch_process = subprocess.Popen(
                [
                    "ros2", "launch", "sllidar_ros2",
                    "sllidar_a2m12_launch.py",
                    "serial_port:=/dev/arduino"
                ],
                preexec_fn=os.setsid
            )
        except Exception:
            pass

def stop_ros2_launch():
    global launch_process
    if launch_process:
        try:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
            launch_process.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(launch_process.pid), signal.SIGKILL)
            except Exception:
                pass
        finally:
            launch_process = None

def launch_camera():
    global camera_process
    if not camera_process and is_camera_connected():
        try:
            camera_process = subprocess.Popen(
                ["python3", camera_script_name],
                preexec_fn=os.setsid
            )
        except Exception:
            pass

def stop_camera():
    global camera_process
    if camera_process:
        try:
            os.killpg(os.getpgid(camera_process.pid), signal.SIGINT)
            camera_process.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(camera_process.pid), signal.SIGKILL)
            except Exception:
                pass
        finally:
            camera_process = None

def watchdog_thread():
    global watchdog_active, launch_process, lidar_should_run

    while True:
        if watchdog_active and lidar_should_run:
            if launch_process is None or launch_process.poll() is not None:
                launch_ros2()
        else:
            # Si no debe estar activo, resetea procesos e intenta no lanzar nada
            pass
        time.sleep(watchdog_retry_delay)

def monitor_system():
    global lidar_should_run, launch_process, camera_process, watchdog_active

    while True:
        if lidar_should_run:
            watchdog_active = True
            # Si el LIDAR está activo, lanza la cámara
            if launch_process and launch_process.poll() is None:
                if not camera_process:
                    launch_camera()
            else:
                # LIDAR no activo -> cámara apagada
                if camera_process:
                    stop_camera()
        else:
            # Apaga todo y desactiva watchdog
            watchdog_active = False
            if launch_process:
                stop_ros2_launch()
            if camera_process:
                stop_camera()

        # Limpia referencias si procesos mueren
        if launch_process and launch_process.poll() is not None:
            launch_process = None
        if camera_process and camera_process.poll() is not None:
            camera_process = None

        time.sleep(0.5)

def on_message(client, userdata, msg):
    global lidar_should_run

    message = msg.payload.decode().strip()
    topic = msg.topic

    if topic == mqtt_topic_lidar:
        if message == '1':
            lidar_should_run = True
        elif message == '0':
            lidar_should_run = False

def setup_mqtt():
    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.on_message = on_message
    client.connect(mqtt_broker)
    client.subscribe(mqtt_topic_lidar)
    return client

if __name__ == "__main__":
    mqtt_client = setup_mqtt()

    threading.Thread(target=monitor_system, daemon=True).start()
    threading.Thread(target=watchdog_thread, daemon=True).start()

    mqtt_client.loop_forever()
