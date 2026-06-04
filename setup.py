from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mecha"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),

        # urdf files
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),

        # rviz files
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),

        # mesh assets
        (os.path.join("share", package_name, "assets"), glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="SO101 LeRobot teleoperation bridge with RViz2 visualization",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "so101_rviz_node = mecha.so101_rviz_node:main",
        ],
    },
)