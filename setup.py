import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_spherical_projection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'cv_bridge',
        'numpy'
    ],
    zip_safe=True,
    maintainer='air',
    maintainer_email='air@todo.todo',
    description='The lidar_spherical_projection package enables the transformation of raw point cloud data from a Velodyne Lidar sensor into a 2D front view image through spherical projection techniques.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_spherical_projection = lidar_spherical_projection.lidar_spherical_projection:main',
        ],
    },
)
