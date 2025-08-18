"""
B4M LLM Navigation API Package

This package provides a REST API interface for Large Language Models to control
robot navigation through the ROS2 Navigation2 stack.

Main components:
- api_server: FastAPI server providing HTTP/JSON interface
- spatial_interpreter: Converts occupancy grids to LLM-friendly text
- nav_controller_node: Bridges API commands to ROS2 Navigation2
- simulated_b4m: Automated navigation for testing without real LLM
"""

__version__ = "1.0.0"
__author__ = "B4M Development Team"
__email__ = "developer@b4m.com"