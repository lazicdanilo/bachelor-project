from setuptools import setup

package_name = 'dipl'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/world_v1.wbt'
]))

data_files.append(('share/' + package_name, [
    'launch/robot_launch.py'
]))

data_files.append(('share/' + package_name, [
    'dipl/driver.py' 
]))


data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files= data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danilo',
    maintainer_email='lazicdanilo47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = dipl.driver:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
