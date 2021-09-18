from setuptools import setup

package_name = 'eurobot_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/robot_webots.urdf',
            'resource/ros2_control.yaml',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/default.wbt', 'worlds/.default.wbproj',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/robot_launch.py',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danilo Lazic',
    maintainer_email='lazicdanilo47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
