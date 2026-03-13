from setuptools import find_packages, setup

package_name = 'roboai_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hristo Bogdanov',
    maintainer_email='hbogdanov@users.noreply.github.com',
    description='RoboAI ROS2 planning prototype package.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'planning_node = roboai_planning.planning_node:main',
        ],
    },
)
