from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot4_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/explore_launch.py')),
        (os.path.join('share', package_name), glob('launch/mapping_launch.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='m.maneet-singh@oth-aw.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_node = turtlebot4_pkg.turtlebot4_node:main',
            'Algorithms= turtlebot4_pkg.Algorithms:main',
            'planner=turtlebot4_pkg.planner:main',
            'control=turtlebot4_pkg.control:main'

        ],
    },
)
