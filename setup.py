"""
Setup script for the drover package
Install in edit mode with: `pip install -e .`
"""

from setuptools import setup, find_packages

setup(
    name='drover',
    version='0.1',
    description="Code for the DRover project",
    author='Ian Burwell',
    maintainer='NUAV',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'opencv-python (>=4.7.0.68)',
        'opencv-contrib-python (>=4.7.0.68)',
        'pymavlink',
        'loguru',
        'navpy',
        'rpi_ws281x',
        'adafruit-circuitpython-neopixel'
    ],
    scripts=[
        'drover/drover_main.py',
        'drover/drover_startup.sh'
    ]  # Provided files can be run from CLI
)
