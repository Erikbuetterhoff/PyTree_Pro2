import os
from glob import glob
from setuptools import setup

package_name = 'test_bt_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action*')),
        (os.path.join('share', package_name, 'subtrees'), glob('subtrees/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jelte',
    maintainer_email='jelte@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_test_node = test_bt_pkg.bt_test_node:main',
            'two_test_node = test_bt_pkg.two_test_node:main',
            'pub_test = test_bt_pkg.pub_test:main',
            'bt_test_sub = test_bt_pkg.bt_test_sub:main',
            'test_action_server = test_bt_pkg.test_action_server:main',
            'wait_action_server = test_bt_pkg.wait_action_server:main',
            'empty_action_server = test_bt_pkg.empty_action_server:main',
            'gui_pub = test_bt_pkg.gui_pub:main',
            'gui_pub2 = test_bt_pkg.gui_pub2:main',
            'gui_pub3 = test_bt_pkg.gui_pub3:main',
        ],
    },
)
