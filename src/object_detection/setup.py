from setuptools import find_packages, setup


setup(
    name="object_detection",
    version="0.0.1",
    packages=find_packages(),
    package_data={"object_detection.detectors": ["models/*"]},
)