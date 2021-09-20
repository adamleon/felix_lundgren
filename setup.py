import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

import launch

package_name = 'felix_lundgren'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.stl')),
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.dae'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adamleon',
    maintainer_email='adam.l.kleppe@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
