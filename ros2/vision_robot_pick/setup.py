from setuptools import find_packages, setup

package_name = 'vision_robot_pick'

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
    maintainer='prof',
    maintainer_email='prof@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_check = vision_robot_pick.layer1_camera_check:main',
            'detector_2d = vision_robot_pick.layer2_detector_2d:main',
            'position_3d = vision_robot_pick.layer3_position_3d:main',
            'position_3d_optimize = vision_robot_pick.layer3_position_3d_optimize:main', 
            'calibration = vision_robot_pick.layer4_calibration:main',
            'capture_xyz = vision_robot_pick.layer3_capture_xyz:main',
            'calibration_solver = vision_robot_pick.layer4_calibration_solver:main',
            'calib_test = vision_robot_pick.calib_test:main',
            'position_objects_3d = vision_robot_pick.position_3d:main',
            'robot_commander = vision_robot_pick.robot_commander:main',
            'ai_agent = vision_robot_pick.ai_agent:main',
            ],
    },
)
