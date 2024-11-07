from setuptools import setup

package_name = 'kitchen_display'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keynmouse',
    maintainer_email='keynmouse@example.com',
    description='Kitchen display node for restaurant system',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'kitchen_display_node = kitchen_display.kitchen_display_node:main',
        ],
    },
)
