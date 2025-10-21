from setuptools import find_packages
from setuptools import setup

setup(
    name='hikvision_interface',
    version='0.0.0',
    packages=find_packages(
        include=('hikvision_interface', 'hikvision_interface.*')),
)
