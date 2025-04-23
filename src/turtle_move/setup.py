from setuptools import find_packages, setup

package_name = 'turtle_move'

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
    maintainer='nathanielrobotics',
    maintainer_email='ngreen26@terpmail.umd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['move = turtle_move.publisher:main', 'read = turtle_move.subscriber:main'
        ],
    },
)
