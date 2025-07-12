import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import LightringLeds, LedColor
import paho.mqtt.client as mqtt
import signal
import sys

class LightRingPublisher(Node):

    def __init__(self):
        super().__init__('light_ring_publisher')
        self.publisher_ = self.create_publisher(LightringLeds, '/cmd_lightring', 10)
        self.timer = None
        self.is_sequence_active = False
        self.is_white = True

        # Inicializar cliente MQTT
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("192.168.8.20", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.mqtt_client.subscribe("Activacion/LED")

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        if message == "1":
            self.activate_sequence()
        elif message == "0":
            self.deactivate_sequence()
            if not self.is_sequence_active:
                self.set_leds_white()

    def activate_sequence(self):
        if not self.is_sequence_active:
            self.is_sequence_active = True
            if self.timer is None:
                self.timer = self.create_timer(0.5, self.timer_callback)

    def deactivate_sequence(self):
        if self.is_sequence_active:
            self.is_sequence_active = False
            if self.timer:
                self.timer.cancel()
                self.timer = None
            self.set_leds_white()

    def timer_callback(self):
        if self.is_sequence_active:
            msg = LightringLeds()
            msg.override_system = True
            if self.is_white:
                msg.leds = [LedColor(red=255, green=255, blue=255)] * 6
            else:
                msg.leds = [LedColor(red=255, green=0, blue=0)] * 6
            self.publisher_.publish(msg)
            self.is_white = not self.is_white

    def set_leds_white(self):
        msg = LightringLeds()
        msg.override_system = True
        msg.leds = [LedColor(red=255, green=255, blue=255)] * 6
        self.publisher_.publish(msg)

    def cleanup(self):
        self.get_logger().info('Apagando cliente MQTT...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

def shutdown_signal_handler(signal, frame, node):
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    light_ring_publisher = LightRingPublisher()

    signal.signal(signal.SIGINT, lambda s, f: shutdown_signal_handler(s, f, light_ring_publisher))
    signal.signal(signal.SIGTERM, lambda s, f: shutdown_signal_handler(s, f, light_ring_publisher))

    try:
        rclpy.spin(light_ring_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        light_ring_publisher.cleanup()
        light_ring_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



