from setuptools import setup

package_name = 'publish_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='IMU Publisher Node using BNO08X.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'imu_publisher = publish_imu.imu_publisher:main',
        ],
    },
)
