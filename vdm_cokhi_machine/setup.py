import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vdm_cokhi_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tannhat',
    maintainer_email='nguyentannhat2298@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc_read_plc_keyence = vdm_cokhi_machine.PC_read_PLC_keyence:main',
            'pc_service_ros = vdm_cokhi_machine.PC_service_ros:main',
            'plc_service_ros = vdm_cokhi_machine.PLC_keyence_service_ros:main'
        ],
    },
)
