from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'load_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anson',
    maintainer_email='maote.liu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'call_load_map_node = load_map.call_load_map:main',
            'set_pose_node = load_map.set_pose:main',
            'nav_srv_node = load_map.nav_srv:main',
        ],
    },
)
