# coding: utf-8
"""
******************************************************************************
*  Copyright 2025 Labforge Inc.                                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License");            *
* you may not use this project except in compliance with the License.        *
* You may obtain a copy of the License at                                    *
*                                                                            *
*     https://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
******************************************************************************
"""
__author__ = "Thomas Reidemeister <thomas@labforge.ca>"
__copyright__ = "Copyright 2025, Labforge Inc."

from glob import glob
import os
import warnings
from pkg_resources import PkgResourcesDeprecationWarning
from setuptools import find_packages, setup

# Silences noisy warnings produced by system packages with non-PEP440 versions.
warnings.filterwarnings("ignore", category=PkgResourcesDeprecationWarning)

PACKAGE_NAME = 'vmd3_radar_driver'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('lib/' + PACKAGE_NAME, glob('vmd3_radar_driver/*.py')),
        # Include configuration files, launch files, etc., here if needed
        (os.path.join('share', PACKAGE_NAME), glob('launch/*_launch.py')),
    ],
    install_requires=[
      'setuptools',
      'numpy'],
    zip_safe=True,
    maintainer='Thomas Reidemeister',
    maintainer_email='thomas@labforge.ca',
    description='ROS 2 driver for the VMD3 mmWave radar sensor.',
    license='Apache License 2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
        'driver = vmd3_radar_driver:main',
        ],
    }
)
