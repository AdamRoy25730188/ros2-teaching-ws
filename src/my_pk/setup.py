from setuptools import find_packages, setup

package_name = 'my_pk'

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
    maintainer='lcas',
    maintainer_email='student@socstech.support',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm_node = my_pk.m_node:main',
            'opencv_bridge = my_pk.opencv_bridge:main',
            'colour_chaser = my_pk.colour_chaser:main'
        ],
    },
)
