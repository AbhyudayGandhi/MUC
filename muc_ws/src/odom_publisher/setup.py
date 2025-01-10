from setuptools import setup

package_name = 'odom_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='agrawalanirudh18@gmail.com',
    description='Odometry publisher for ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher_node = odom_publisher.odom_publisher_node:main',
        ],
    },
)

