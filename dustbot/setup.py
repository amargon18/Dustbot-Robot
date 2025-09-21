from setuptools import setup
from glob import glob
import os

package_name = 'dustbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=['dustbot'],
    install_requires=['setuptools', 'dustbot_interfaces'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='youremail@domain.com',
    description='Dustbot simulation with direction control',
    license='Apache-2.0',
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/dustbot']),
    ('share/dustbot', ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('src/dustbot/launch/*.py')),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_node = dustbot.world_node:main',
            'robot_node = dustbot.robot_node:main',
            'dustbot_position_publisher = dustbot.dustbot_position_publisher:main',
            'garbage_position_publisher = dustbot.garbage_position_publisher:main',
        ],
    },
)
