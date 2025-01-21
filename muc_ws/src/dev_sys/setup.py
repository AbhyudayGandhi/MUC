from setuptools import find_packages, setup

package_name = 'dev_sys'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dev_sys.launch.py']),

        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gnanasai',
    maintainer_email='pendyalagnanasai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'odom_publisher = odom_publisher.odom_publisher:main',
        'publish_pwm_values = publish_pwm_values.publish_pwm_values:main',
        'subscribe_encoder_values = subscribe_encoder_values.subscribe_encoder_values:main',
        ],
    },
)
