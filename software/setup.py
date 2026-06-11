from distutils.core import setup

setup(
    name='ir_panel',
    python_requires='>=3.8',
    author='Martin Privat',
    version='0.0.1',
    packages=['ir_panel'],
    license='Creative Commons Attribution-Noncommercial-Share Alike license',
    description='Remote control video projectors',
    long_description=open('README.md').read(),
    install_requires=[
        "pyserial",
        "numpy"
    ]
)