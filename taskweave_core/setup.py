from setuptools import find_packages, setup

PACKAGE = 'taskweave_core'

setup(
    name=PACKAGE,
    version='0.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', ['launch/taskweave.launch.py']),
        ('bin', ['bin/taskweave']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Rzepczyk',
    maintainer_email='f.rzepczyk@gmail.com',
    description='taskweave core: config, planners, the task handler and the skill server base class.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'task_handler = taskweave_core.task_handler:main',
        'skill_server = taskweave_core.skill_server:main',
        'operator_joy = taskweave_core.operator_joy:main',
        'teach = taskweave_core.teach:main',
        'speed_gate = taskweave_core.speed_gate:main',
        'taskweave = taskweave_core.cli:entry',
    ]},
)
