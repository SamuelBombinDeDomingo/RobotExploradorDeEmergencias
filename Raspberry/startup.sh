#!/bin/bash

# Configura la dirección IP y puerto del broker MQTT
BROKER_IP="192.168.8.20"
BROKER_PORT=1883

# Esperar a que el broker MQTT esté disponible
echo "Esperando que el broker MQTT esté disponible en $BROKER_IP:$BROKER_PORT..."
while ! nc -z "$BROKER_IP" "$BROKER_PORT"; do
    echo "El broker MQTT no está disponible. Reintentando en 5 segundos..."
    sleep 5
done
echo "El broker MQTT está disponible. Continuando con el script..."

# Comprobar si el entorno virtual existe antes de activarlo
VENV_PATH="/home/RobotExploradorDeEmergencias/Robot/venv"
if [ -d "$VENV_PATH" ]; then
    echo "Activando el entorno virtual..."
    source "$VENV_PATH/bin/activate"
else
    echo "El entorno virtual no existe en $VENV_PATH. Verifique la ruta."
    exit 1
fi

# Configuración de ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ws/install/setup.sh

# Validar si los scripts Python existen antes de ejecutarlos
for script in CCS811.py RSSI.py Lanzador.py Servo.py si7021.py; do
    if [ ! -f "/home/RobotExploradorDeEmergencias/$script" ]; then
        echo "El script /home/RobotExploradorDeEmergencias/$script no existe. Abortando."
        exit 1
    fi
done

# Iniciar los scripts en segundo plano
echo "Iniciando los scripts..."
python3 /home/RobotExploradorDeEmergencias/CCS811.py &
python3 /home/RobotExploradorDeEmergencias/RSSI.py &
python3 /home/RobotExploradorDeEmergencias/Lanzador.py &
python3 /home/RobotExploradorDeEmergencias/Servo.py &
python3 /home/RobotExploradorDeEmergencias/si7021.py &

# Lanzar el archivo de ROS 2
ros2 launch create3_republisher create3_republisher_launch.py republisher_ns:=/ robot_ns:=/Robot&

# Mantener el script en ejecución
echo "El script de inicio está funcionando. Presione Ctrl+C para salir."
while true; do
    sleep 60
done
