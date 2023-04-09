from setuptools import setup

package_name = 'object_tracker'

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
    maintainer='ice',
    maintainer_email='notandinotandi@gmail.com',
    description='Track colored objects',
    license='All rights reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracker = object_tracker.object_tracker:main'
        ],
    },
)
