from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'api_handler'

setup(
    name=package_name,
    version='0.0.0',
    # CHANGE THIS LINE:
    # packages=find_packages(exclude=['test']),
    packages=[package_name], # <--- CHANGE TO THIS
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='ayushsinghayush1@gmail.com',
    description='API call handler for the bin picking cell',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_handler_node = api_handler.api_handler_node:main',
            'door_handle_node = api_handler.door_handle_node:main',
            'stack_light_node = api_handler.stack_light_node:main',
            'emergency_button_node = api_handler.emergency_button_node:main', # <-- ADD THIS LINE
        ],
    },
)