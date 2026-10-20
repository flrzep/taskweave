from glob import glob

from setuptools import find_packages, setup

PACKAGE = 'taskweave_hmi'

setup(
    name=PACKAGE,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    package_data={PACKAGE: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/web', glob('taskweave_hmi/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Florian Rzepczyk',
    maintainer_email='f.rzepczyk@gmail.com',
    description='Web HMI for taskweave: live mission status, control and a validated mission editor.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['hmi_server = taskweave_hmi.server:main']},
)
