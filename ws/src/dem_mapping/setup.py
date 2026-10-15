from setuptools import setup

package_name = 'dem_mapping'

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
    description='CPU-only incremental elevation mapping for ROS 2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dem_node = dem_mapping.dem_node:main',
            'gnss_align = dem_mapping.gnss_align:main',
            'georef_export = dem_mapping.georef_export:main',
        ],
    },
)
