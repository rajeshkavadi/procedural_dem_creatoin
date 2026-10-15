from setuptools import setup

package_name = 'dem_viz'

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
    description='Live heatmap visualisation of the incremental DEM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heatmap = dem_viz.heatmap_node:main',
        ],
    },
)
