from setuptools import setup

package_name = 'flair_image_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Gautier',
    maintainer_email='nicogautier07@gmail.com',
    description='Minimal ROS2 node that receives FLAIR camera images over TCP and republishes them as sensor_msgs/Image.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_receiver_node = flair_image_bridge.image_receiver_node:main',
            'optical_flow_demo_node = flair_image_bridge.optical_flow_demo_node:main',
            'dataset_recorder_node = flair_image_bridge.dataset_recorder_node:main',
        ],
    },
)
