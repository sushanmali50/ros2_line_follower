from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'line_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sushan',
    maintainer_email='sushan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_line = line_following.test_line:main',
            'pid_control = line_following.pid_control:main',
            'motor_control = line_following.motor_control:main',
            'contour_test = line_following.contour_test:main',
        ],
    },
)
