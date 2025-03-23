'''
code for ONR LUCI Project
'''
from setuptools import setup

setup(
    name='onr_luci',
    version='0.1',
    packages=['onr_luci'],
    install_requires=[
        'numpy',
        'scipy',
        'casadi',
        'imgui',
        'glfw',
        'PyOpenGL',
        'matplotlib',
        'dill',
        'pyubx2',
        'pyserial',
        'moteus',
        ]
)
