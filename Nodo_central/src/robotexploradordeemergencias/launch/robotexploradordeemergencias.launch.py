from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import socket
import time

def wait_for_mqtt_broker(host, port, timeout=5):
    while True:
        try:
            with socket.create_connection((host, port), timeout):
                break
        except (socket.timeout, ConnectionRefusedError):
            time.sleep(5)

def generate_launch_description():
    wait_for_mqtt_broker('192.168.8.20', 1883)

    pkg_path = FindPackageShare('robotexploradordeemergencias').find('robotexploradordeemergencias')
    scripts = ['Audio.py', 'Camara.py', 'Joystick.py', 'Luces.py', 'Microfono.py', 'ModoDeNavegacion.py',
               'MoviminetoAcople_Desacople.py', 'Robot.py', 'Sirena.py', 'Transformada.py','Puente.py']
    
    script_paths = [PathJoinSubstitution([pkg_path, 'launch', script]) for script in scripts]
    processes = [
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen'
        ) for script_path in script_paths
    ]
    
    node_red = ExecuteProcess(
        cmd=['node-red'],
        output='screen'
    )
    
    mosquitto = ExecuteProcess(
        cmd=['/usr/sbin/mosquitto', '-d'],
        output='screen'
    )
    
    ros2_launch_files = [
        'dock_description.launch.py',
        'robot_description.launch.py',
    ]
    
    ros2_processes = [
        ExecuteProcess(
            cmd=['ros2', 'launch', 'irobot_create_common_bringup', launch_file],
            output='screen'
        ) for launch_file in ros2_launch_files
    ]
    
    delayed_processes = [
        TimerAction(period=5.0, actions=[proc]) for proc in processes
    ]
    
    ld = LaunchDescription()
    ld.add_action(mosquitto)
    ld.add_action(node_red)
    
    for proc in delayed_processes:
        ld.add_action(proc)
    
    for proc in ros2_processes:
        ld.add_action(proc)
    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=node_red, on_exit=[
            ExecuteProcess(cmd=['pkill', '-f', 'node-red'])
        ])
    ))
    
    return ld 
