from setuptools import setup

package_name = 'robot_action_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Robot action demo',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'move_robot_server = robot_action_demo.move_robot_server:main',
            'move_robot_client = robot_action_demo.move_robot_client:main',
        ],
    },
)