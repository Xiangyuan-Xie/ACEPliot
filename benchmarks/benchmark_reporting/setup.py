from setuptools import find_packages, setup


package_name = "benchmark_reporting"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="Xiangyuan Xie",
    maintainer_email="dragonboat_xxy@163.com",
    description="ACEPliot benchmark reporting nodes and metric utilities.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "benchmark_reporter = benchmark_reporting.reporter_node:main",
        ],
    },
)
