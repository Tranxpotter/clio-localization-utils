from setuptools import find_packages, setup

package_name = 'localization_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abc',
    maintainer_email='michaelyclaw@gmail.com',
    description='Localization utils for metacam data grounding',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_odom_node = localization_utils.global_odom_node:main',
            'pose_estimate_remapper = localization_utils.pose_estimate_remapper:main', 
            'sensor_frame_corrector = localization_utils.sensor_frame_corrector:main',
            'static_odom_publisher = localization_utils.static_odom_publisher:main',
        ],
    },
)
