from setuptools import find_packages, setup

package_name = "shadowhound_skills"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pillow", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="ShadowHound Team",
    maintainer_email="developer@shadowhound.local",
    description="Vision and perception skills for ShadowHound autonomous robot",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "skill_test_node = shadowhound_skills.skill_test_node:main"
        ],
    },
)
