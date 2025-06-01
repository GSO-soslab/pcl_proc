from setuptools import setup
import os
from glob import glob

package_name = 'pcl_proc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure the 'launch' files are correctly handled
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        # Ensure the 'config' YAML files are correctly handled
        (os.path.join('share', package_name, 'config'),glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'pcl_proc'),glob(os.path.join('pcl_proc', 'path_utils.py'))),
    ],

    package_data={
            'mvp_msgs' : ['srv/*.srv'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tony Jacob',
    maintainer_email='tony.jacob@uri.edu ',
    description='Package to generate and utilize path',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcl_filter_node = pcl_proc.filter:main',
            'path_generator = pcl_proc.path_gen:main',
            'waypoint_admin = pcl_proc.wp_admin:main'
        ],
    },
)
