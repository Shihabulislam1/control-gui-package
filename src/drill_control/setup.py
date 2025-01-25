from setuptools import find_packages, setup

package_name = 'drill_control'

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
    maintainer='merr2',
    maintainer_email='mdshihabulislam.mte.ruet@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drill = drill_control.drill_motor_subscriber:main',
            'arm = drill_control.arm_control:main',
        ],
    },
)
