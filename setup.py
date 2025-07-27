from setuptools import find_packages, setup
from glob import glob

package_name = 'dual_camera_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonmd',
    maintainer_email='moonmd@github.com',
    description='Dual‑camera ROS 2 launch package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        'launch': [
            'dual_cameras = dual_camera_launch.dual_cameras:generate_launch_description',
        ],
    },
)

