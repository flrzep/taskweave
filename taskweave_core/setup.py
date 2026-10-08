from setuptools import find_packages, setup

PACKAGE = 'taskweave_core'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Rzepczyk',
    maintainer_email='f.rzepczyk@gmail.com',
    description='taskweave core: config, planners, the task handler and the skill server base class.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
