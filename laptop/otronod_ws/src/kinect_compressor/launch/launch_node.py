import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo 1: KinectImageRepublisher
        Node(
            package='kinect_compressor',  # Nombre de tu paquete
            executable='kinect_image_republisher',  # El nombre del nodo (sin extensión .py)
            name='kinect_image_republisher_node',
            output='screen',
            parameters=[],  # Parámetros si es necesario
            remappings=[('/kinect/image_raw', '/kinect/image_raw')]  # Remapeo de tópicos si es necesario
        ),
        
        # Nodo 2: PersonRescue
        Node(
            package='kinect_compressor',  # Nombre de tu paquete
            executable='person_rescue',  # El nombre del nodo (sin extensión .py)
            name='person_rescue_node',
            output='screen',
            parameters=[{'image_width': 300}],  # Parámetros si es necesario
            remappings=[]  # Remapeo de tópicos si es necesario
        )
    ])
