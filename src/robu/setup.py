import os
from setuptools import setup
from glob import glob

package_name = 'robu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/rviz', ['rviz/tb3_cartographer.rviz']),
        ('share/' + package_name + '/config', ['config/turtlebot3_lds_2d.lua']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael.lieschnegg',
    maintainer_email='lieschnegg@lilatec.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld = robu.helloworld:main',
            'remotectrl = robu.remotectrl:main',
            'remotectrl_listener = robu.remotectrl_listener:main',
            'remotectrl_sus = robu.remotectrl_sus:main',
            'ex03_obstacle_avoidance_simple=robu.ex03_obstacle_avoidance_simple:main',
            'ex03_obstacle_avoidance_simple_mapping=robu.ex03_obstacle_avoidance_simple_mapping:main',
            'ex02_remotectrl = robu.ex02_remotectrl:main',
            'ex10_wallfollower = robu.ex10_wallfollower:main',
            'ex10_wallfollower_li = robu.ex10_wallfollower_li:main',
            'mypublisher = robu.publisher_member_function:main',
            'mysubscriber = robu.subscriber_member_function:main',
        ],
    },
)
