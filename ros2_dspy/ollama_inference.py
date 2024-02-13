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
from ros2_dspy_msgs.action import QA

DOCKER_COMPOSE_PATH = get_package_share_directory('ros2_dspy') + '/.docker/docker-compose-ollama.yaml'

class BasicQA(dspy.Signature):
    """Answer common sense questions."""
    question = dspy.InputField(desc="a question about solving a real-world manipulation task")
    answer = dspy.OutputField(desc="a succinct answer to the question")

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
        subprocess.run(f'ollama pull {model}', shell=True, check=True)
        subprocess.run(f'docker compose -f {DOCKER_COMPOSE_PATH} up -d --no-recreate', shell=True, check=True)

        # setup DSPy to interact with local Ollama server
        ollama_model = dspy.OllamaLocal(
                model=model,
                model_type='chat'
                )
        dspy.settings.configure(lm=ollama_model)
        self.qa_predictor= dspy.Predict(BasicQA)

        # initialize action server for ollama inference
        self._ollama_predict_action_server = ActionServer(
            self,
            QA,
            'ollama_predict',
            self.ollama_qa_callback,
        )

    def ollama_qa_callback(self, goal_handle):
        """Callback for ollama_predict action server."""
        self.logger.info('Received ollama_predict action request.')
        # TODO: add feedback to action server
        pred = self.qa_predictor(question=goal_handle.request.question)
        self.logger.info(f'Answer: {pred.answer}')
        result = QA.Result()
        result.answer = pred.answer
        return result
            
def main(args=None):
    rclpy.init(args=args)
    ollama_inference_node = OllamaInferenceNode()
    rclpy.spin(ollama_inference_node)
    ollama_inference_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
