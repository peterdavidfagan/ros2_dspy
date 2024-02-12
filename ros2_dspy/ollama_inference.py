"""Performing inference with Ollama and DSPy."""

import os
import sys
import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import dspy
#from ros2_dspy_msgs.action import DSPY

DOCKER_COMPOSE_PATH = get_package_share_directory('ros2_dspy') + '/.docker/docker-compose-ollama.yaml'

class OllamaInferenceNode(Node):
    """Node for performing inference with Ollama and DSPy."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('ollama_inference_node')
        self.logger = self.get_logger()       
        
        # config parameters
        self.declare_parameter('ollama_model', 'llama2')
        model = self.get_parameter('ollama_model').get_parameter_value().string_value

        # start ollama inference server
        self.logger.info('Starting Ollama inference server...')
        subprocess.run(f'docker compose -f {DOCKER_COMPOSE_PATH} up -d --no-recreate', shell=True, check=True)
        subprocess.run(f'docker exec ollama-inference-ollama-1 ollama run {model}', shell=True, check=True)

        # setup DSPy to interact with local Ollama server
        ollama_model = dspy.OpenAI(
                api_base='http://localhost:11434/v1/', 
                api_key='ollama', 
                model=model,
                stop='\n\n', 
                model_type='chat'
                )
        dspy.settings.configure(lm=ollama_model)


        # initialize action server for ollama inference
        #self._ollama_predict_action_server = ActionServer(
        #    self,
        #    DSPY,
        #    'ollama_predict',
        #    self._ollama_predict_callback,
        #    )

        def _ollama_predict_callback(self, goal_handle):
            """Callback for ollama_predict action server."""
            self.logger.info('Received ollama_predict action request.')
            

def main(args=None):
    rclpy.init(args=args)
    ollama_inference_node = OllamaInferenceNode()
    rclpy.spin(ollama_inference_node)
    ollama_inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
