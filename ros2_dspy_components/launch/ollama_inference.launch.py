from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node


def generate_launch_description():
    
    ollama_params = {
            "ollama_node": ParameterBuilder("ros2_dspy")
            .yaml("config/ollama.yaml")
            .to_dict()
            }

    ollama_node = Node(
            package="ros2_dspy",
            executable="ollama_inference",
            parameters=[ollama_params],
            output="both",
            )

    return LaunchDescription([ollama_node])
