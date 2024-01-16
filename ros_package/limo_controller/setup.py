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
            "keyboard_controller = limo_controller.keyboard_controller:main",
            "apply_control = limo_controller.apply_control:main"
        ],
    },
)
