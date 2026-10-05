import os
from glob import glob

from setuptools import find_packages, setup


package_name = "block_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="limopro",
    maintainer_email="limopro@todo.todo",
    description="OpenCV-only RGB-D block detector for ROS 2 Humble.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "block_detector = block_detector.block_detector_node:main",
        ],
    },
)
