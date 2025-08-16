from setuptools import setup, find_packages

package_name = 'multi_robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_controller.launch.py']),
        ('share/' + package_name + '/params', ['params/multi_robot_controller.yaml']),
        ('share/' + package_name + '/tasks', ['tasks/tasks_example.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='you@example.com',
    description='Multi-robot manager for Nav2-based robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_robot_controller = multi_robot_controller.multi_robot_controller:main',
        ],
    },
)

