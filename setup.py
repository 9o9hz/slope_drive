import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'slope_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j',
    maintainer_email='seonjuhan1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'imu_orientation_node = slope_drive.imu_orientation_node:main',
            'bev_projection_node = slope_drive.bev_projection_node:main',
            'edge_lane_node = slope_drive.edge_lane_node:main',
            'path_planner_node = slope_drive.path_planner_node:main',
            'pure_pursuit_node = slope_drive.pure_pursuit_node:main',
        ],
    },
)