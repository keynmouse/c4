from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),  # 서비스 파일 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimyunwoo',
    maintainer_email='kimyunwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'client=test3.service_client6:main',
        'server=test3.service_server6:main',
        ],
    },
)
