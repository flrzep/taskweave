from setuptools import find_packages, setup

PACKAGE = 'taskweave_bringup'

setup(
    name=PACKAGE,
    version='0.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', ['launch/taskweave_bringup.launch.py', 'launch/taskweave_nav2.launch.py',
                                          'launch/taskweave_moveit.launch.py']),
        ('share/' + PACKAGE + '/config', ['config/nav2_params.yaml']),
    ],
    entry_points={'console_scripts': ['clock_guard = taskweave_bringup.clock_guard:main']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Rzepczyk',
    maintainer_email='f.rzepczyk@gmail.com',
    description='taskweave bring-up: launch files for taskweave, the HMI, Nav2 and MoveIt of any robot pack.',
    license='TBD',
    tests_require=['pytest'],
)
