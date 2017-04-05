from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'openag_brain',
        'openag_brain.commands',
        'openag_brain.peripherals'
    ],
    package_dir={'': 'src'},
)

setup(**setup_args)
