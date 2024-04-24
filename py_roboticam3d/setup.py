from setuptools import find_packages, setup

package_name = 'py_roboticam3d'

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
    maintainer='clem',
    maintainer_email='clemence.dovillers@gmail.com',
    description='Acquisition et reconstruction',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = py_roboticam3d.service_member_function:main',
        'client = py_roboticam3d.client_member_function:main',
        ],
    },
)
