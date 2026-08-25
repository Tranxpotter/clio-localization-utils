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
            'pose_estimate_remapper = localization_utils.pose_estimate_remapper:main', 
            'static_odom_publisher = localization_utils.static_odom_publisher:main', 
            'sensor_frame_corrector = localization_utils.sensor_frame_corrector:main'
        ],
    },
)
