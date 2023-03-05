from setuptools import setup

package_name = 'quadrotor_simulator_ros'

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
    maintainer='kshitijgoel',
    maintainer_email='kshitijgoel16061995@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadrotor_simulator_node = quadrotor_simulator_ros.quadrotor_simulator_node:main'
        ],
    },
)
