from setuptools import find_packages, setup

package_name = 'TAC_Manipulator'

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
    maintainer='halo',
    maintainer_email='abhiveersingh2007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'subscribe = TAC_Manipulator.subscribe:main',    
            'publish = TAC_Manipulator.publish:main',
            'pub_exp = TAC_Manipulator.publish_experiment:main',
            'updated_pub = TAC_Manipulator.updated_publisher:main',
            'latest_publiser  = TAC_Manipulator.latest_publisher:main',
            'bridge = TAC_Manipulator.bridge:main',
            'sub_py = TAC_Mannipulator.sub_py.main',
        ],
    },
)
