from setuptools import find_packages, setup

package_name = 'ld2410_ble_sensor'

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
    maintainer='elsabot',
    maintainer_email='horton.rscott@gmail.com',
    description='LD2410 BLE Human presense sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ld2410_ble = ld2410_ble_sensor.ld2410_ble:main'
        ],
    },
)
