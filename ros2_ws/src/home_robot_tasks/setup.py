from setuptools import find_packages, setup

setup(
    name="home_robot_tasks",
    version="0.1.0",
    packages=find_packages(),
    package_data={"home_robot_tasks": ["web/*.html", "web/*.css", "web/*.js"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/home_robot_tasks"]),
        ("share/home_robot_tasks", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="dasom",
    maintainer_email="noreply@github.com",
    description="Hardware-independent household task requests and planning.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "plan_fetch = home_robot_tasks.cli:main",
            "replay_fetch = home_robot_tasks.replay_cli:main",
            "simulate_robot = home_robot_tasks.simulate_cli:main",
            "robot_gateway = home_robot_tasks.gateway:main",
            "simulate_search = home_robot_tasks.search_simulator:main",
        ]
    },
)
