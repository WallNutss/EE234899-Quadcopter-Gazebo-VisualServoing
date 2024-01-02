from setuptools import setup

package_name = 'camera_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'ibvs_pid = camera_control.ibvs_control_PID:main',
            'ibvs_smc = camera_control.ibvs_control_SMC:main'
        ],
    },
)
