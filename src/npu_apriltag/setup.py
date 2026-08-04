import os

from setuptools import find_packages
from setuptools import setup


package_name = 'npu_apriltag'


def package_files(directory):
    return [
        os.path.join(directory, name)
        for name in sorted(os.listdir(directory))
        if os.path.isfile(os.path.join(directory, name)) and not name.startswith('.')
    ] if os.path.isdir(directory) else []


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', package_files('launch')),
        ('share/' + package_name + '/config', package_files('config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NPU IUS Lab',
    maintainer_email='npu-ius-lab@users.noreply.github.com',
    description='Marker detection configuration and normalization of detector results to a world-frame marker pose.',
    license='YOLO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_adapter = npu_apriltag.detection_adapter:main',
        ],
    },
)
