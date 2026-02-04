from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slope_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일(.launch.py)을 설치 경로에 포함시키는 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j',
    maintainer_email='seonjuhan1@gmail.com',
    description='Slope navigation robot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_orientation_node = slope_drive.imu_orientation_node:main',
            'bev_projection_node = slope_drive.bev_projection_node:main',
            'edge_lane_node = slope_drive.edge_lane_node:main',
            'skeletonization_node = slope_drive.skeletonization_node:main',
            'branch_pruning_node = slope_drive.branch_pruning_node:main',
            'path_planner_node = slope_drive.path_planner_node:main',
            # 시각화 노드 (visualizer_node.py 파일을 만들었을 때 실행 가능)
            'visualizer_node = slope_drive.visualizer_node:main',
        ],
    },
)