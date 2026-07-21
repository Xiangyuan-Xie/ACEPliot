from glob import glob
import os

from setuptools import find_packages, setup


package_name = "arm_motion_benchmark"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="Xiangyuan Xie",
    maintainer_email="dragonboat_xxy@163.com",
    description="Arm motion workload and base-stability benchmark.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_motion_benchmark = arm_motion_benchmark.node:main",
        ],
    },
)
