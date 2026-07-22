from setuptools import find_packages, setup

package_name = 'kinova_haptic_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haptic',
    maintainer_email='haptic@todo.todo',
    description='Teleoperation bridge for chi-haptics pneumatic interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_torque_pub = kinova_haptic_teleop.fake_torque_pub:main',
            'kinova_haptic_bridge = kinova_haptic_teleop.kinova_haptic_bridge:main',
            'kinova_haptic_bridge_sim = kinova_haptic_teleop.kinova_haptic_bridge_sim:main',
        ],
    },
)
