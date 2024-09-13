from setuptools import setup, find_packages

package_name = "robot_system_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=[
        "setuptools",
        "launch",
        "launch_ros",
        "launch_testing",
        "launch_testing_ros",
        "pytest",
    ],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO",
    description="Robot system tests for one singular robot",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
