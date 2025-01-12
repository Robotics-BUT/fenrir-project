from setuptools import find_packages, setup

package_name = 'buttons_handler'

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
    description='Button handler package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'buttons_handler = buttons_handler.buttons_handler:main',
        ],
    },
)
