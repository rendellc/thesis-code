from setuptools import setup

package_name = 'report_utils'

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
    maintainer_email='rendellc@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker = report_utils.publisher_member_function:main",
            "listener = report_utils.listener:main",
            "bagsaver = report_utils.bagsaver:main"
        ],
    },
)
