import os

from setuptools import find_packages, setup

PACKAGE = 'taskweave_hmi'


def tree(folder: str) -> list[tuple[str, list[str]]]:
    """data_files entries for every file below ``folder`` (data_files takes files, not folders: web/vendor/ nests)."""
    return [(f'share/{PACKAGE}/{os.path.relpath(path, PACKAGE)}', [os.path.join(path, file) for file in files])
            for path, _, files in os.walk(folder) if files]


setup(
    name=PACKAGE,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    package_data={PACKAGE: ['web/**/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        *tree(f'{PACKAGE}/web'),
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
