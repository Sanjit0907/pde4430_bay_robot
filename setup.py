"""setup.py - installs the pde4430_bay_robot package (URDF, launch files, Python nodes)."""
from glob import glob
from setuptools import setup

package_name = 'pde4430_bay_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PDE4430 Student',
    maintainer_email='student@example.com',
    description='PDE4430 bay-pushing robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_controller = pde4430_bay_robot.arm_controller:main',
        ],
    },
)
