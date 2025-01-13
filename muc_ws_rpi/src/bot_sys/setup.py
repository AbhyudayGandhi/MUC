from setuptools import find_packages, setup

package_name = 'bot_sys'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/'+ package_name+ '/launch', ['launch/bot_sys.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='agrawalanirudh18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'motor_control = motor_control.motor_control:main',
	'encoder_reader = encoder_reader.encoder_reader:main',
	'publish_imu = publish_imu.publish_imu:main',
        ],
    },
)
