from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nabil',
    maintainer_email='nabil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "led_control_simple = my_robot_controller.led_control_simple:main",
            "led_state_reader = my_robot_controller.led_state_reader:main",
            "diff_tf = my_robot_controller.diff_tf_odom:main",
            "diff_tf_v2 = my_robot_controller.diff_tf_v2:main",
            "diff_tf_pid_v3 = my_robot_controller.diff_tf_pid_v3:main",
            "diff_tf_pid_v4 = my_robot_controller.diff_tf_pid_v4:main"
           
        ],
    },
)
