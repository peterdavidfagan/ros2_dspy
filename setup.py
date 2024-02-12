from setuptools import setup

package_name = 'ros2_dspy'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ('share/' + package_name + '/launch', ['launch/ollama_inference.launch.py']),
     ('share/' + package_name + '/config', ['config/ollama.yaml']),
     ('share/' + package_name + '/.docker', ['.docker/docker-compose-ollama.yaml']),
     ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Peter David Fagan',
 maintainer_email='peterdavidfagan@gmail.com',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     "console_scripts": [
         "ollama_inference = ros2_dspy.ollama_inference:main",
        ],
    },
)

from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "ros2_dspy_parameters", # python module name for parameter library
  "config/ollama.yaml", # path to input yaml file
)
