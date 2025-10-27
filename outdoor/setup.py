import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'outdoor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='americogomes1@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "find_a_person    = outdoor.find_a_person.lets_bora:main",
            "movement_mapping = outdoor.mapping.movement_mapping:main",
            "photos_mapping   = outdoor.mapping.photos_mapping:main",
        ],
    },
)
