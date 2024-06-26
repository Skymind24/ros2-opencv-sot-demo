import os
from glob import glob
from setuptools import setup

package_name = 'ros2_opencv_sot_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='skymind0204@gmail.com',
    description='Opencv SOT Demo with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v4l2_camera_node = v4l2_camera.v4l2_camera_node:main',
            'tracker = ros2_opencv_sot_demo.tracker:main',
            'detector = ros2_opencv_sot_demo.detector:main'
        ],
    },
)
