from setuptools import find_packages, setup
import os
import glob
package_name = 'grond_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/joystick_demo.launch.py']),
        ('share/' + package_name, ['launch/keyboard_demo.launch.py']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devkit',
    maintainer_email='devkit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = grond_utils.test_pub:main',
            'key_publisher = grond_utils.key_publisher:main',
            'joystick_publisher = grond_utils.joystick_commander:main',

        ],
    },
)
