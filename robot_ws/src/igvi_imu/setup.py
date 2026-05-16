from setuptools import find_packages, setup

package_name = "igvi_imu"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IGVI",
    maintainer_email="igvi@example.com",
    description="Online IMU bias calibration utilities for IGVI robot sensors",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_bias_calibrator = igvi_imu.imu_bias_calibrator:main",
        ],
    },
)
