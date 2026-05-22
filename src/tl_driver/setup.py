from setuptools import setup
from glob import glob
import os

package_name = 'tl_driver'

# data files to install so that ros2 can find launch/config and the swig lib
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/tl_driver']),
    ('share/' + package_name, ['package.xml']),
]

# include launch and config files if present
for p in glob('launch/*.py'):
    data_files.append(('share/' + package_name + '/launch', [p]))
for p in glob('config/*'):
    data_files.append(('share/' + package_name + '/config', [p]))

# include the native shared object if present in lib/
if os.path.exists('lib/_nrc_host.so'):
    data_files.append(('share/' + package_name + '/lib', ['lib/_nrc_host.so']))

setup(
    name=package_name,
    version='1.0.0',
    packages=['tl_driver', 'tl_driver.lib'],
    package_dir={'': 'src'},
    package_data={'tl_driver.lib': ['_nrc_host.*', 'nrc_interface.py']},
    data_files=data_files,
    include_package_data=True,
    zip_safe=False,
    install_requires=['setuptools'],
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 driver for TL series robotic arm (Python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tl_driver_node = tl_driver.tl_driver_node:main',
        ],
    },
)
