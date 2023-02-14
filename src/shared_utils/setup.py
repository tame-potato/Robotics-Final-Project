# setup file for shared_utils package

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['shared_utils'],
    package_dir={'': 'src'}
)

setup(**d)
