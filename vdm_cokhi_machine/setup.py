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
        ],
    },
)
