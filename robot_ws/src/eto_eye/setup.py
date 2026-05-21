import os
from glob import glob
from setuptools import find_packages, setup

package_name = "eto_eye"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vmlab",
    maintainer_email="jan5303151@gmail.com",
    description="YOLO detection node for ETO robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detector_node = eto_eye.detector_node:main",
            "semantic_memory_node = eto_eye.semantic_memory_node:main",
        ],
    },
)
