from setuptools import setup
from glob import glob
import os

package_name = 'tl_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages', ['resource/' + package_name]
        ),

        (
            'share/' + package_name, ['package.xml']
        ),

        # 安装 launch 文件
        (
            os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')
        ),

        # 安装 json 文件（标准 config/ 目录）
        (
            os.path.join('share', package_name, 'config'), ['config/saved_points.json']
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='jack',
    maintainer_email='jack@example.com',

    description='TL robot examples',
    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'servoj_trajectory_playback = tl_example.servoj_trajectory_playback:main',
            'queue_trajectory_playback = tl_example.queue_trajectory_playback:main',
        ],
    },
)