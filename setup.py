## catkin_python_setup(): makes `bf_ros_bridge` importable from src/
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["bf_ros_bridge"],
    package_dir={"": "src"},
)

setup(**setup_args)
