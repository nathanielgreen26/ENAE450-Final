import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world*'))),
        (os.path.join('share', package_name, 'models','maze_0', 'maze_1', 'maze_2'), glob(os.path.join('models','maze_0', 'maze_1', 'maze_2','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anusha',
    maintainer_email='anusha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_controller = maze_simulation.maze_controller:main',
        ],
    },
)
