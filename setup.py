from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_args = generate_distutils_setup(
    packages=['proprietary_template'],
    # below may want to append robopress to src; currently works as is though
    package_dir={'': 'src'}
)
setup(**setup_args)