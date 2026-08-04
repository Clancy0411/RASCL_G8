from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rascl_wp3_ss26_group8'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'trajectories'), glob('trajectories/*')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 8',
    maintainer_email='group8@example.com',
    description='WP3 motion planning and robot control application package for RASCL group 8.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wp3_tsk1 = rascl_wp3_ss26_group8.wp3_tsk1:main',
            'wp3_tsk2 = rascl_wp3_ss26_group8.wp3_tsk2:main',
        ],
    },
)
