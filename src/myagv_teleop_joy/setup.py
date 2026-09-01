import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = 'myagv_teleop_joy'


setup(

    name=package_name,

    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools',
        'evdev',
    ],

    zip_safe=True,

    maintainer='alex',

    maintainer_email='ninec9369@gmail.com',

    description=(
        'Teleoperacion omnidireccional del myAGV '
        'con un mando Bluetooth (Xbox Series)'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [

            (
                'bluetooth_gamepad_teleop = '
                'myagv_teleop_joy.'
                'bluetooth_gamepad_teleop:main'
            ),

        ],
    },
)
