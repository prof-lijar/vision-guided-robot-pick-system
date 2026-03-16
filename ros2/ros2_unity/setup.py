from setuptools import setup

package_name='ros2_unity'

setup(
name=package_name,
version='0.0.0',
packages=[package_name],
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
install_requires=['setuptools'],
zip_safe=True,
maintainer='your_name',
maintainer_email='your@email.com',
description='FR5 Unity bridge',
license='TODO',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'joint_bridge = ros2_unity.joint_bridge:main',
'sdk_version_node = ros2_unity.sdk_version_node:main',
'state_publisher = ros2_unity.state_publisher:main',
'cmd_server = ros2_unity.cmd_server:main',
'state_publisher_socket = ros2_unity.state_publisher_socket:main',
'jog_server = ros2_unity.jog_server:main',
        ],
    },
)
