from setuptools import setup

package_name = 'py_tree_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py-trees', 'py-trees-ros'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='dein.email@beispiel.com',
    description='Ein einfaches ROS2 Paket mit py-trees und py-trees-ros',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = py_tree_pkg.main:main',
        ],
    },
)
