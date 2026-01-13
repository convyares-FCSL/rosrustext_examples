from glob import glob
from setuptools import setup

package_name = "lesson_06_lifecycle_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install params/config files
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Lesson 05 (Python): parameters.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lesson_06_lifecycle_publisher = lesson_06_lifecycle_py.publisher_node:main",
            "lesson_06_lifecycle_subscriber = lesson_06_lifecycle_py.subscriber_node:main",
        ],
    },
)
