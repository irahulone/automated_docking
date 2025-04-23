from setuptools import find_packages, setup

package_name = 'control_directives'

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
            'director = control_directives.docking_director:main',
            'fcmc = control_directives.fluid_charge_management_controller:main',
        ],
    },
)

