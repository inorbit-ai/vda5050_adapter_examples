from glob import glob
from setuptools import setup

package_name = 'vda5050_tb3_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + "/config", glob('config/*.yaml')),
        ('share/' + package_name + "/maps", glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Leandro Pineda',
    author_email="leandro@inorbit.ai",
    maintainer='TBD',
    maintainer_email='tbd@tbd.com',
    description='VDA5050 Turtlebot3 adapter example',
    license='BSD Clause 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb3_adapter = vda5050_tb3_adapter.tb3_adapter:main'
        ],
    },
)
