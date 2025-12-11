import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'defendagentattack_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanimn',
    maintainer_email='hanimn@uchicago.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reward-simulate = defendagentattack_2.reward_simulator:main',
            'multi-agent = defendagentattack_2.multiagent:main',
            'fight-now = defendagentattack_2.fightnow:main',
            'rl-move = defendagentattack_2.rl_agent_nodes:main',
        ],
    },
)
