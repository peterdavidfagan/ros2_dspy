"""Performing inference with Ollama and DSPy."""

import os
import sys
import subprocess

import rclpy
from rclpy.node import Node

import dspy


class OllamaInferenceNode(Node):
    """Node for performing inference with Ollama and DSPy."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('ollama_inference_node')
       
        # declare node parameters
        self.declare_parameter("ollama_model", rclpy.Parameter.Type.STRING, "ollama2")
        self.ollama_model = self.get_parameter('ollama_model').get_parameter_value().string_value
        
        # start ollama inference server
        self.logger.info('Starting Ollama inference server...')
        subprocess.run('docker compose -f ../.docker/docker-compose-ollama.yml up -d', shell=True, check=True)
        subprocess.run('docker exec -it ollama ollama run llama2', shell=True, check=True)

        # initialize action server for ollama inference


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = OllamaInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
