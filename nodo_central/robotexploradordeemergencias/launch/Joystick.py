import serial
import serial.tools.list_ports
import threading
import time

import geometry_msgs.msg
import rclpy
import paho.mqtt.client as mqtt

# Variables globales
active = False  # Estado de activación por MQTT
last_button_state = 0  # Estado anterior del botón
speed = 1.1  # Velocidad lineal por defecto
turn = 1.1   # Velocidad angular por defecto
ser = None   # Objeto serial para Arduino

modo_navegacion = "0"  # Valor por defecto del modo navegación MQTT
ultimo_twist_cmd_vel_twist = None  # Último mensaje recibido en /cmd_vel_twist


def on_connect(client, userdata, flags, rc):
    client.subscribe("Casa")
    client.subscribe("Velocidad")  # Suscribirse al tópico de velocidad
    client.subscribe("Modo_Navegacion")  # Suscribirse a modo navegación


def on_message(client, userdata, msg):
    global active, speed, turn, modo_navegacion
    topic = msg.topic
    payload = msg.payload.decode()

    if topic == "Casa":
        if payload == "1":
            active = True
        elif payload == "0":
            active = False
    elif topic == "Velocidad":
        try:
            new_speed = float(payload)
            speed = new_speed
            turn = new_speed
        except ValueError:
            pass
    elif topic == "Modo_Navegacion":
        modo_navegacion = payload


def mqtt_loop(client):
    client.loop_start()


def reconnect_arduino():
    """Intenta reconectar con el Arduino en los puertos disponibles."""
    global ser
    while True:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            try:
                ser = serial.Serial(port.device, 9600, timeout=1)
                return
            except serial.SerialException:
                pass
        time.sleep(2)


def cmd_vel_twist_callback(msg):
    global ultimo_twist_cmd_vel_twist
    ultimo_twist_cmd_vel_twist = msg


def main():
    global ser, last_button_state, modo_navegacion, ultimo_twist_cmd_vel_twist

    try:
        rclpy.init()

        node = rclpy.create_node('teleop_twist_keyboard')

        # MQTT setup
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect("192.168.8.20", 1883, 60)

        mqtt_thread = threading.Thread(target=mqtt_loop, args=(mqtt_client,))
        mqtt_thread.start()

        # Parámetros para usar Twist o TwistStamped
        stamped = node.declare_parameter('stamped', False).value
        frame_id = node.declare_parameter('frame_id', '').value
        TwistMsg = geometry_msgs.msg.TwistStamped if stamped else geometry_msgs.msg.Twist

        pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

        # Suscriptor a /cmd_vel_twist (tipo Twist)
        node.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel_twist',
            cmd_vel_twist_callback,
            10)

        spinner = threading.Thread(target=rclpy.spin, args=(node,))
        spinner.start()

        # Inicializar twist_msg al comienzo
        twist_msg = TwistMsg()
        if stamped:
            twist_msg.header.frame_id = frame_id

        # Intento de conectar con Arduino
        reconnect_arduino()

        while rclpy.ok():
            try:
                # Leer las coordenadas y el estado del botón desde el puerto serie
                line = ser.readline().decode('utf-8').strip()
                data = line.split(',')
                if len(data) < 3:
                    continue

                xValue = int(data[0])
                yValue = int(data[1])
                buttonState = int(data[2])

                # Detectar flanco de subida en el botón
                if buttonState == 1 and last_button_state == 0:
                    mqtt_client.publish("Boton", "1")

                last_button_state = buttonState

                # Verificar si el nodo está activo según el estado de MQTT
                if active:
                    # Ajustar los movimientos según las coordenadas del joystick y el botón
                    if xValue < 400 and 600 < yValue:
                        x, y, z, th = 0.0, 0.0, 0.0, -1.0
                    elif 400 < xValue and 600 < yValue:
                        x, y, z, th = 0.0, 0.0, 0.0, 1.0
                    elif 800 < xValue and yValue < 400:
                        x, y, z, th = 1.0, 0.0, 0.0, 1.0
                    elif xValue < 300 and yValue < 400:
                        x, y, z, th = 1.0, 0.0, 0.0, -1.0
                    elif 300 < xValue < 800 and yValue == 0:
                        x, y, z, th = 1.0, 0.0, 0.0, 0.0
                    else:
                        if modo_navegacion == "3" and ultimo_twist_cmd_vel_twist is not None:
                            if stamped:
                                twist_msg.twist = ultimo_twist_cmd_vel_twist
                                twist_msg.header.stamp = node.get_clock().now().to_msg()
                                pub.publish(twist_msg)
                            else:
                                pub.publish(ultimo_twist_cmd_vel_twist)
                            continue  # Saltar publicar con valores 0
                        else:
                            continue  # No publicar nada y saltar el resto del ciclo

                    # Si llegamos aquí significa que sí queremos publicar x,y,z,th
                    if stamped:
                        twist_msg.header.stamp = node.get_clock().now().to_msg()
                        twist = twist_msg.twist
                        twist.linear.x = x * speed
                        twist.linear.y = y * speed
                        twist.linear.z = z * speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * turn
                        pub.publish(twist_msg)
                    else:
                        twist = geometry_msgs.msg.Twist()
                        twist.linear.x = x * speed
                        twist.linear.y = y * speed
                        twist.linear.z = z * speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * turn
                        pub.publish(twist)

            except serial.SerialException:
                reconnect_arduino()

            time.sleep(0.1)

    except Exception:
        pass

    finally:
        # Publicar TwistMsg de parada
        twist = twist_msg.twist if stamped else twist_msg
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()
        mqtt_client.disconnect()


if __name__ == '__main__':
    main()

