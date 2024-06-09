from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'custom_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/nav_srv.yaml']),
        # Ensure all python scripts are installed
        ('share/' + package_name + '/custom_nav', [
            'custom_nav/manager.py',
            'custom_nav/nav_srv.py',
            'custom_nav/sample_navigator.py',
            'custom_nav/srv_client_test.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='csl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = custom_nav.srv_client_test:main',
            'nav_srv = custom_nav.nav_srv:main', 
            'manager = custom_nav.manager:main',  
        ],
    },
)
