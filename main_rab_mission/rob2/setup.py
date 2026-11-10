from setuptools import setup, find_packages

setup(
    name='pi-walk',
    author="Sarah",
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'pipuck-sdk @ git+https://github.com/OmriPer/pipuck-sdk.git'
    ]
)

