import os
from glob import glob
from setuptools import setup

package_name = 'px4_single_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*.[yma]*'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools', 'pytest'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'velocity_control = px4_single_plan.velocity_control:main',
                'offboard_waypoint_runner = px4_single_plan.offboard_waypoint_runner:main',
                'start_mission_gate = px4_single_plan.start_mission_gate:main',
                'offboard_safety = px4_single_plan.safety_cli:main',
        ],
    },
)
