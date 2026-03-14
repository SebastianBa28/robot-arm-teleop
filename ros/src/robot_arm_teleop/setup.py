from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_arm_teleop'

otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
