from glob import glob
from setuptools import find_packages, setup

package_name = 'team_xx'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    #install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CyPR - University of Malaga',
    maintainer_email='jgmonroy@uma.es',
    description='Python starter package for CyPR Team Project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'racer = team_xx.racer:main',
            'warrior = team_xx.warrior:main',
        ],
    },
)