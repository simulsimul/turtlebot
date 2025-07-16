"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'webots_ros2_turtlebot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/hardware_launch.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/ros2control.yml',
    'resource/nav2_params.yaml',
    'resource/data.csv',
    'resource/data2.csv'
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='2025.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch', 'numpy', 'scikit-learn'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TurtleBot3 Burger robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigator = webots_ros2_turtlebot.auto_navigator:main',
            'wall_follower_rule = webots_ros2_turtlebot.wall_follower_rule:main',
            'wall_follower_pid = webots_ros2_turtlebot.wall_follower_pid:main',
            'wall_follower_ml = webots_ros2_turtlebot.wall_follower_ml:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
