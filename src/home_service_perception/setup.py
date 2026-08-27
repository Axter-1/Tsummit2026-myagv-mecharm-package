from setuptools import find_packages, setup

from glob import glob
import os


package_name = "home_service_perception"


setup(

    name=package_name,

    version="0.0.1",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[

        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name
            ]
        ),

        (
            "share/" + package_name,
            [
                "package.xml"
            ]
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch"
            ),

            glob(
                "launch/*.launch.py"
            )
        ),
    ],

    install_requires=[
        "setuptools"
    ],

    zip_safe=True,

    maintainer="alex",

    maintainer_email="alex@example.com",

    description=(
        "Perception nodes for "
        "myAGV Home Service"
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [

            "aruco_detector_node = "
            "home_service_perception."
            "aruco_detector_node:main",

        ],
    },
)
