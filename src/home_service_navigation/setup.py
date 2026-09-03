import os

from glob import glob

from setuptools import find_packages, setup


package_name = "home_service_navigation"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alex",
    maintainer_email="alex@example.com",
    description="Scan sanitizer and maze runner for the myAGV",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_sanitizer_node = "
            "home_service_navigation.scan_sanitizer_node:main",
            "maze_runner_node = "
            "home_service_navigation.maze_runner_node:main",
        ],
    },
)
