from setuptools import find_packages, setup

package_name = 'i2c_handler'

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
    maintainer='Adam Ligocki',
    maintainer_email='adam.ligocki@vut.cz',
    description='rgb led handler package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'i2c_handler = i2c_handler.i2c_handler:main',
        ],
    },
)
