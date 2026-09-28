from setuptools import find_packages, setup

package_name = 'franka_kdl_broadcaster'

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
    maintainer='iitgn-robotics',
    maintainer_email='debojit.das@iitgn.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "franka_kdl_fk_node = franka_kdl_broadcaster.franka_kdl_fk_node:main",
        ],
    },
)
