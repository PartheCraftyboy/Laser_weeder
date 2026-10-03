from setuptools import setup
from glob import glob

package_name = 'jim_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone',
    maintainer_email='you@example.com',
    description='Field simulation, oracle detector and targeting for Jim.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oracle_detector = jim_sim.oracle_detector:main',
            'targeting_node = jim_sim.targeting_node:main',
        ],
    },
)
