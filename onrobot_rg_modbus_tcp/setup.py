## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['onrobot_rg_modbus_tcp'],
    package_dir={'': 'src'},
    requires=['rospy', 'pymodbus==2.1.0'],
)

setup(**setup_args)
