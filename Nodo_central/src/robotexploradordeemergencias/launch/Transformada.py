import subprocess

def launch_static_transform():
    # Definir los comandos a ejecutar
    command1 = [
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '--x', '0', '--y', '0', '--z', '0.1',  # Posición (x, y, z)
        '--frame-id', 'base_link', '--child-frame-id', 'laser',  # Frame ids
        '--roll', '0', '--pitch', '0', '--yaw', '3.14159'  # Rotación (roll, pitch, yaw), 180 grados sobre Z
    ]

    try:
        # Ejecutar ambos comandos en paralelo
        process1 = subprocess.Popen(command1)
       

        # Esperar a que ambos procesos terminen (esto puede ser opcional dependiendo de cómo quieras manejarlos)
        process1.wait()
     

    except subprocess.CalledProcessError:
        pass  # No imprimir nada en caso de error

if __name__ == "__main__":
    launch_static_transform()

