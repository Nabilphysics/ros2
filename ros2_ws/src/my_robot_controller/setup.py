from setuptools import setup

package_name = 'my_robot_controller'
submodules = "my_robot_controller"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Razwanul Haque Nabil',
    maintainer_email='nabil@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diff_drive_robot = my_robot_controller.diff_drive_robot:main"
           
        ],
    },
)
