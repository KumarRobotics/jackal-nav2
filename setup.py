from glob import glob
from setuptools import find_packages, setup


package_name = "jackal_nav2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools", "PyYAML", "matplotlib"],
    zip_safe=True,
    maintainer="Ankit Prabhu",
    maintainer_email="pra.ankiict@gmail.com",
    description=(
        "Standalone ROS 2 sensor and Nav2 autonomy bringup for the DCIST Jackal."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "cmd_vel_to_joy = jackal_nav2.cmd_vel_to_joy:main",
            "goto_nav2 = jackal_nav2.goto_nav2:main",
            "motion_stats = jackal_nav2.motion_stats:main",
        ],
    },
)
