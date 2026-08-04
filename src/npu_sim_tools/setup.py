import os

from setuptools import find_packages
from setuptools import setup


package_name = 'npu_sim_tools'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NPU IUS Lab',
    maintainer_email='npu-ius-lab@users.noreply.github.com',
    description='Mock external pose publisher, test fixtures, and README command-reference validator.',
    license='YOLO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_mock_pose = npu_sim_tools.publish_mock_pose:main',
            'validate_readme_commands = npu_sim_tools.validate_readme_commands:main',
        ],
    },
)
