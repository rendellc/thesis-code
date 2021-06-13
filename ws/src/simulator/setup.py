from setuptools import setup

package_name = 'simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cale',
    maintainer_email='email',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawn_vehicle = simulator.spawn_vehicle:main",
            "spawn_vehicle_offcenter = simulator.spawn_vehicle_offcenter:main",
        ],
    },
)
