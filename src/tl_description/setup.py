from setuptools import setup
from glob import glob
import os

package_name = 'tl_description'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# install directories
for pattern, dest in [
    ('launch/*.launch.py', 'launch'),
    ('urdf/*', 'urdf'),
    ('meshes/**/*', None),  # handled via recursive below
    ('rviz/*', 'rviz'),
    ('config/*', 'config'),
]:
    if dest:
        for p in glob(pattern, recursive=False):
            data_files.append(('share/' + package_name + '/' + dest, [p]))

# Handle meshes recursively
for root, dirs, files in os.walk('meshes'):
    if files:
        install_dir = 'share/' + package_name + '/' + root
        data_files.append((install_dir, [os.path.join(root, f) for f in files]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='my_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={},
)
