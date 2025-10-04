from setuptools import find_packages, setup
import os
from glob import glob

package_name = "shadowhound_mission_agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=[
        "setuptools",
        "fastapi>=0.104.0",
        "uvicorn>=0.24.0",
        "websockets>=12.0",
        "pydantic>=2.0.0",
    ],
    zip_safe=True,
    maintainer="ShadowHound Team",
    maintainer_email="developer@shadowhound.local",
    description="ShadowHound mission agent integrating DIMOS autonomous capabilities",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mission_agent = shadowhound_mission_agent.mission_agent:main",
        ],
    },
)
