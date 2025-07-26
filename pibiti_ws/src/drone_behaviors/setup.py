from setuptools import find_packages, setup

package_name = 'drone_behaviors'
submodules = [
    'drone_behaviors.flight',
    'drone_behaviors.conditions',
    'drone_behaviors.subtrees',
    'drone_behaviors.commander',
    'drone_behaviors.camera',
]
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + submodules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='gabrielbfranca@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
