import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
import paho.mqtt.client as mqtt
import signal
import sys

class SirenController(Node):
    def __init__(self):
        super().__init__('siren_controller')
        self.publisher_ = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.siren_active = False
        self.siren_timer = None
        
        self.mqtt_client = mqtt.Client(client_id='ros2_siren_controller', clean_session=True, protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        self.mqtt_client.connect('192.168.8.20', 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe("Sirena")

    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        if message == '1' and not self.siren_active:
            self.siren_active = True
            self.start_siren()
        elif message == '0' and self.siren_active:
            self.siren_active = False
            self.stop_siren()

    def start_siren(self):
        if self.siren_timer is None:
            self.siren_timer = self.create_timer(0.45, self.publish_siren)

    def stop_siren(self):
        if self.siren_timer is not None:
            self.siren_timer.cancel()
            self.siren_timer = None
            self.deactivate_siren()

    def publish_siren(self):
        if self.siren_active:
            siren_msg = AudioNoteVector()
            siren_msg.append = False
            siren_msg.notes = [
                AudioNote(frequency=660, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=660, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=660, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=660, max_runtime=Duration(sec=0, nanosec=250000000)),
                AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=250000000)),
            ]
            self.publisher_.publish(siren_msg)

    def deactivate_siren(self):
        siren_msg = AudioNoteVector()
        siren_msg.append = False
        siren_msg.notes = []
        self.publisher_.publish(siren_msg)

def shutdown_signal_handler(signal, frame, siren_controller):
    siren_controller.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    siren_controller = SirenController()

    signal.signal(signal.SIGTERM, lambda s, f: shutdown_signal_handler(s, f, siren_controller))

    try:
        rclpy.spin(siren_controller)
    except KeyboardInterrupt:
        pass
    finally:
        siren_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


