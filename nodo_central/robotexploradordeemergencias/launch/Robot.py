import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import DockStatus
import paho.mqtt.client as mqtt

class BatteryStateListener(Node):

    def __init__(self):
        super().__init__('battery_state_listener')
        
        # Configurar QoS para ROS 2
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Suscribirse al tema /battery_state
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            qos_profile)

        # Suscribirse al tema /dock_status
        self.create_subscription(
            DockStatus,
            '/dock_status',
            self.dock_status_callback,
            qos_profile)

        # Configurar conexión MQTT
        self.mqtt_client = mqtt.Client(client_id='battery_state_listener', protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.connect_to_mqtt()

    def connect_to_mqtt(self):
        try:
            self.mqtt_client.connect("192.168.8.20", 1883)
            self.mqtt_client.loop_start()
        except Exception as e:
            pass  # No imprimir nada

    def on_mqtt_connect(self, client, userdata, flags, reasonCode, properties=None):
        pass  # No imprimir nada

    def on_mqtt_disconnect(self, client, userdata, reasonCode, properties=None):
        pass  # No imprimir nada

    def battery_state_callback(self, msg):
        # Filtrar los mensajes por frame_id
        if msg.header.frame_id == 'base_link':
            return  # Ignorar mensajes con frame_id 'base_link'

        # Publicar el porcentaje de batería en MQTT
        battery_percentage = msg.percentage * 100.0
        self.mqtt_client.publish('Bateria', payload=f"{battery_percentage:.4f}", qos=1)

    def dock_status_callback(self, msg):
        # Publicar 0 si dock_visible es False, 1 si es True
        visible_status = '1' if msg.dock_visible else '0'
        self.mqtt_client.publish('Visible', payload=visible_status, qos=1)

def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    battery_state_listener = BatteryStateListener()

    try:
        rclpy.spin(battery_state_listener)
    except KeyboardInterrupt:
        pass
    finally:
        battery_state_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
