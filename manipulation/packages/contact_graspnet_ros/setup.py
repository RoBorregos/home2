import os
from glob import glob
from setuptools import setup

package_name = "contact_graspnet_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoBorregos",
    maintainer_email="luiss.benvenuto@gmail.com",
    description="ROS 2 wrapper for Contact-GraspNet",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "contact_graspnet_node = contact_graspnet_ros.contact_graspnet_node:main",
        ],
    },
)
