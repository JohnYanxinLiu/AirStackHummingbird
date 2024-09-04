from setuptools import setup, find_packages

package_name = 'robot_system_tests'

setup(
 name=package_name,
 version='0.0.0',
 packages=find_packages(),
data_files=[
('share/' + package_name, ['package.xml']),
('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'my_node = my_py_pkg.my_node:main'
     ],
   },
)