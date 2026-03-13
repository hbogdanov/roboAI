from setuptools import setup

package_name = "roboai_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ivan",
    maintainer_email="ivan@example.com",
    description="RoboAI planning node scaffold.",
    license="MIT",
    entry_points={"console_scripts": ["planning_node = roboai_planning.planning_node:main"]},
)
