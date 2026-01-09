from setuptools import setup
import os
from glob import glob

package_name = 'lesson_py_template'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        # (Optional) If you have launch files later, uncomment this:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Template Python ROS 2 node package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The executable name = package_name.module:function
            'lesson_node = lesson_py_template.node:main',
        ],
    },
)