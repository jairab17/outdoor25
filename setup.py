from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'outdoor25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dronkab',
    maintainer_email='dronkab.unaq@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            "px4_driver = outdoor25.px4_driver:main",
            "move_drone = outdoor25.move_drone:main"
        ],
    },
)