from setuptools import find_packages, setup

package_name = 'roboai_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hbogdanov',
    maintainer_email='hbogdanov@todo.todo',
    description='RoboAI control node',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'control_node = roboai_control.control_node:main',
        ],
    },
)
