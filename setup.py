
from glob import glob
import os
from setuptools import setup

package_name = "panda_teleop"

# build a list of the data files
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append(("share/" + package_name, glob("launch/*.launch.py", recursive=True)))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('description/', data_files)

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        package_name + '.scripts',
        package_name + '.scripts.models',
        package_name + '.scripts.rbd',
        package_name + '.scripts.rbd.idyntree'],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    author="njpalomo",
    author_email="npalomo@student.ethz.ch",
    maintainer="njpalomo",
    maintainer_email="npalomo@student.ethz.ch",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Panda demo in ROS2.",
    long_description="""\
Panda demo in ROS2.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "panda_keyboard_control = \
                panda_teleop.panda_keyboard_control:main",
        ],
    },
)
