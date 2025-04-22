from setuptools import find_packages, setup

package_name = 'arduino_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toughbook',
    maintainer_email='toughbook@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spin_arduino_cs = arduino_pkg.arduino_wraper_central_station:main',
            'spin_arduino_ri = arduino_pkg.arduino_wraper_rover_i:main',
        ],
    },
)
