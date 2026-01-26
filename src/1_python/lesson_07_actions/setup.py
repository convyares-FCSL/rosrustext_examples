from glob import glob
from setuptools import setup

package_name = "lesson_07_actions"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + '.action_server', package_name + '.action_client'],
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
    description="Lesson 07 (Python): actions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lesson_07_action_server = lesson_07_actions.action_server.node:main",
            "lesson_07_action_client = lesson_07_actions.action_client.action_client:main",
        ],
    },
)
