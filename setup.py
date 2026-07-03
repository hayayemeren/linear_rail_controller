from setuptools import find_packages, setup
import os
from glob import glob

# UPDATED: Package name changed
package_name = 'linear_rail_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='Control node for custom linear rails',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # UPDATED: Points to the new python file and creates the executable name
            'absolute_homer_left = linear_rail_controller.absolute_homer_left:main',
            'demo = linear_rail_controller.demo:main'
        ],
    },
)
