from setuptools import find_packages, setup

package_name = 'mybot_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='disaster',
    maintainer_email='disaster@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    keywords=['ROS'],
    entry_points={
        'console_scripts': [
		'teleop = mybot_description.teleop:main'
        ],
    },
)
