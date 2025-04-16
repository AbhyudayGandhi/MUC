from setuptools import setup

package_name = 'cmd_vel_to_stm32'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 node to send /cmd_vel data to STM32 via serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_stm32_node = cmd_vel_to_stm32.cmd_vel_to_stm32_node:main',
        ],
    },
)

