from setuptools import setup
import os
from glob import glob

package_name = 'ball_balancer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_controller = ball_balancer.lqr_controller:main',
        ],
    },
)
