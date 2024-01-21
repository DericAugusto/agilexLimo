from setuptools import find_packages, setup

package_name = 'limo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'pylimo',
        'tkinter',
        'threading',
        'multiprocessing',
        'pynput',  # Add your non-ROS dependencies here
        # Add any other package dependencies here
    ],
    zip_safe=True,
    maintainer='dericaugusto',
    maintainer_email='dericaugustofs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "apply_control = limo_controller.node_apply_control:main",
            "clicking_controller = limo_controller.node_clicking_controller:main",
            "keyboard_controller = limo_controller.node_keyboard_controller:main",
            "trajectory_controller = limo_controller.node_trajectory_controller:main",
            "turtle_monitor = limo_controller.node_turtle_monitor:main",
            # name_of_executable = package_name.file_name:main
        ],
    },
)
