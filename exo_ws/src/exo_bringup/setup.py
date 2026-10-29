
from setuptools import find_packages, setup

package_name = 'exo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            ['exo_bringup/launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kassim',
    maintainer_email='kassim@todo.todo',
    description='Exoskeleton bringup and launch files',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
