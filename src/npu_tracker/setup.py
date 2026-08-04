import os

from setuptools import find_packages
from setuptools import setup


package_name = 'npu_tracker'


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
    description='Flight mission, drone following, and marker tracking task nodes for the course simulation.',
    license='YOLO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = npu_tracker.mission:main',
            'follow_drone_tf = npu_tracker.follow_drone_tf:main',
            'publish_follow_goal = npu_tracker.publish_follow_goal:main',
            'tag_tracker = npu_tracker.tag_tracker:main',
        ],
    },
)
