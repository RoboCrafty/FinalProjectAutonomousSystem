from setuptools import setup

package_name = 'lantern_hunting_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan',
    maintainer_email='ryan@todo.todo',
    description='Pilot node for automated lantern hunting.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'hunter_node = lantern_hunting_pkg.hunter_node:main',
        ],
    },
)
