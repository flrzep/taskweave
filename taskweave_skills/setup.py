from setuptools import find_packages, setup

PACKAGE = 'taskweave_skills'

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
    description='Generic, robot-agnostic taskweave skills (wait, navigate_to, go_to_point).',
    license='TBD',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['tcp_jog = taskweave_skills.tcp_jog:main']},
)
