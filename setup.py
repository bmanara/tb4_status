import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'tb4_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian_2025',
    maintainer_email='bmacraze@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb4_status_node = tb4_status.tb4_status_node:main',
            'tb4_api_node = tb4_status.tb4_api_node:main',
            'tb4_ws_node = tb4_status.tb4_ws_node:main'
        ],
    },    
)
