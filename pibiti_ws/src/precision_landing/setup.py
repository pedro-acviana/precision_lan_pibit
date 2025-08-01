from setuptools import find_packages, setup

package_name = 'precision_landing'

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
    maintainer='pedrobook',
    maintainer_email='pedroacviana@outlook.com',
    description='Precision landing package for drones using computer vision',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = precision_landing.main:main',
            'takeoff_tree = precision_landing.takeoff_tree:main'
        ],
    },
)
