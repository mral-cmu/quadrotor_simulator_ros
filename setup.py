import os
from setuptools import setup
from glob import glob

package_name = 'quadrotor_simulator_ros'

setup(
   name=package_name,
   packages=[package_name],
   data_files=[
       ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'assets'), glob('assets/*'))
       ]
)
