from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diag_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irmak',
    maintainer_email='irmak.kazanci@std.yildiz.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'diag_float32ma_node = diag_pkg.slam.diag_float32ma_node:main',
        'diag_grid_node = diag_pkg.slam.diag_grid_node:main',
        'diag_image_node = diag_pkg.perception.diag_image_node:main',
        'diag_int32_node = diag_pkg.control.diag_int32_node:main',
        'diag_int32s_node = diag_pkg.slam.diag_int32s_node:main',
        'diag_int32ma_node = diag_pkg.slam.diag_int32ma_node:main',
        'diag_laserscan_node = diag_pkg.slam.diag_laserscan_node:main',
        'diag_markerarray_node = diag_pkg.slam.diag_markerarray_node:main',
        'diag_odom_node = diag_pkg.slam.diag_odom_node:main',
        'diag_path_node = diag_pkg.navigation.diag_path_node:main',
        'diag_pointc2_node = diag_pkg.slam.diag_pointc2_node:main',
        'diag_posearray_node = diag_pkg.navigation.diag_posearray_node:main',
        'diag_posestamped_node = diag_pkg.slam.diag_posestamped_node:main',
        'diag_string_node = diag_pkg.perception.diag_string_node:main',
        'diag_int8_node = diag_pkg.control.diag_int8_node:main',
        'diag_int8s_node = diag_pkg.slam.diag_int8s_node:main',
        'dashboard_node = diag_pkg.dashboard_node:main',
        'emergency_brake_node = diag_pkg.emergency.emergency_brake_node:main',
        'velodyne_recovery_launcher_node = diag_pkg.emergency.velodyne_recovery_launcher_node:main',
        'loop_detector_node = diag_pkg.emergency.loop_detector_node:main',
        ],
    },
)
