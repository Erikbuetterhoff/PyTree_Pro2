import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jelte',
    maintainer_email='jelte.wierper@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'test_node = ur_test_pkg.test_node:main',
        ],
    },
)
