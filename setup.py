from setuptools import setup
import os
from glob import glob

package_name = "fish_delivery"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ethan",
    maintainer_email="ethank2784@gmail.com",
    description="Slow arm extension to hand the caught fish to the child for the Stretch fishing game.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fish_deliver = fish_delivery.fish_deliver:main",
        ],
    },
)
