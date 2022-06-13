from setuptools import setup
import os
from glob import glob

package_name = 'atcart8_nav2_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='A demo package of ATCart8 with Nav2',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'check_lidar = atcart8_nav2_demo.check_lidar:main',
        'small_object_stop = atcart8_nav2_demo.small_object_stop:main'
        ],
    },
)
