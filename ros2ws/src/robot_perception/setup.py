from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models',
         glob(os.path.join('robot_perception', 'models', '*.pt'))),
    ],
    include_package_data=True,
    package_data={package_name: ['models/*.pt']},
    install_requires=['setuptools', 'opencv-python', 'ultralytics>=8.0.0'],
    zip_safe=True,
    maintainer='hbeydoun',
    maintainer_email='hbeydoun@udmercy.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'detect_object = robot_perception.stream_perception:main',
        ],
    },
)
