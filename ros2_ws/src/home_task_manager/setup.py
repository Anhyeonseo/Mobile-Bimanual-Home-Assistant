from setuptools import find_packages, setup

setup(
    name="home_task_manager",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/home_task_manager"]),
        ("share/home_task_manager", ["package.xml", "README.md"]),
        ("share/home_task_manager/examples", ["examples/pick_gateway.json"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    maintainer="Home Assistant contributors",
    maintainer_email="noreply@github.com",
    description="Simulation-only ROS2 Pick task manager.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "task_manager = home_task_manager.node:main",
        "demo = home_task_manager.demo:main",
        "mock_skills = home_task_manager.mock_skills:main",
        "gateway_client = home_task_manager.gateway_client:main",
    ]},
)
