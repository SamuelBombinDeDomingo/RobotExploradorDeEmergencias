import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock
import paho.mqtt.client as mqtt

broker_address = "192.168.8.20"
topic = "Activacion"
acople_topic = "Acople"

class DockUndockActionClient(Node):
    def __init__(self):
        super().__init__('dock_undock_action_client')
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.processing = False

    def send_dock_goal(self):
        goal_msg = Dock.Goal()
        self.dock_client.wait_for_server()
        future = self.dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.dock_goal_response_callback)

    def send_undock_goal(self):
        goal_msg = Undock.Goal()
        self.undock_client.wait_for_server()
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_result("0")
            self.processing = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_result_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_result("0")
            self.processing = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def dock_result_callback(self, future):
        result = future.result().result
        result_message = "1" if result.is_docked else "0"
        self.publish_result(result_message)
        self.processing = False

    def undock_result_callback(self, future):
        result = future.result().result
        result_message = "1" if not result.is_docked else "0"
        self.publish_result(result_message)
        self.processing = False

    def publish_result(self, message):
        mqtt_client.publish(acople_topic, message)

def on_message(client, userdata, message):
    payload = message.payload.decode("utf-8").strip()

    if payload == "1" and not node.processing:
        node.processing = True
        node.send_undock_goal()

    elif payload == "0" and not node.processing:
        node.processing = True
        node.send_dock_goal()

mqtt_client = mqtt.Client()
mqtt_client.connect(broker_address, 1883, 60)
mqtt_client.subscribe(topic)
mqtt_client.on_message = on_message

rclpy.init()
node = DockUndockActionClient()

mqtt_client.loop_start()

rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
mqtt_client.loop_stop()

