import os
from glob import glob
from setuptools import setup


package_name = 'image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share','image_subscriber','launch'), glob(os.path.join('launch','*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wallnuts',
    maintainer_email='wallnuts@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_display = image_subscriber.image_display_node:main',
            'pose_display = image_subscriber.image_pose_display_node:main'
        ],
    },
)
