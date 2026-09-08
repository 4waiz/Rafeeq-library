from glob import glob
import os

from setuptools import setup

package_name = 'rafeeq_solution'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafeeq Team',
    maintainer_email='bridgesoftwareteam@gmail.com',
    description='Rafeeq Library Robot — ERC 2026 Phase 1 solution.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = rafeeq_solution.perception_node:main',
            'navigation_node = rafeeq_solution.navigation_node:main',
            'manipulation_node = rafeeq_solution.manipulation_node:main',
            'mission_node = rafeeq_solution.mission_node:main',
            'tune_arm = rafeeq_solution.tune_arm:main',
        ],
    },
)
