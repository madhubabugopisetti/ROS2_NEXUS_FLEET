from setuptools import find_packages, setup

package_name = 'nexus_controls'

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
    maintainer='madhu',
    maintainer_email='madhubabugopisetti27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_arm = nexus_controls.drive_arm:main',
            'drive_car_arm = nexus_controls.drive_car_arm:main',
            'drive_car = nexus_controls.drive_car:main',
        ],
    },
)
