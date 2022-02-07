from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['abstract_arm'],
    package_dir={'': 'src'}
)

setup(**d)