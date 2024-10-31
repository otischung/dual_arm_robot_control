from setuptools import find_packages, setup

package_name = 'arm_control'

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
    maintainer='otischung',
    maintainer_email='p76124605@gs.ncku.edu.tw',
    description='This project aims to control the dual-arm robot with 6 joints on the arm and 5 joints on the hand.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_reader = arm_control.arm_reader:main',
            'arm_writer = arm_control.arm_writer:main',
            'control_panel = arm_control.control_panel:main'
        ],
    },
)
