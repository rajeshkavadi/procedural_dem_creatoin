from setuptools import setup

package_name = 'dem_replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajesh',
    maintainer_email='user@example.com',
    description='Timestamp-preserving ROS 2 bag replay and sensor stream tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay = dem_replay.replay_node:main',
            'imu_relay = dem_replay.imu_relay_node:main',
            'stream_monitor = dem_replay.stream_monitor_node:main',
            'inspect_bag = dem_replay.inspect_bag:main',
            'bag_convert = dem_replay.bag_convert:main',
            'record_odom = dem_replay.record_odom:main',
        ],
    },
)
