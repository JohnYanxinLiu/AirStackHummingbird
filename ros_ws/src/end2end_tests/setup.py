from setuptools import setup, find_packages

package_name = "end2end_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", ["launch/isaac_test_env_bringup.xml"]),
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
    description="Tests in simulation for full systems",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
