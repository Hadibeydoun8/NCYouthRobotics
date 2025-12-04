from setuptools import setup, find_packages

package_name = 'robot_command_node'

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
    maintainer='hbeydoun',
    maintainer_email='hbeydoun@todo.todo',
    description='Python nodes for robot_command_node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'talker = robot_command_node.publisher_member_function:main',
        ],
    },
)
