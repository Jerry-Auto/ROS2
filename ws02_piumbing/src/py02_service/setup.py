from setuptools import find_packages, setup

package_name = 'py02_service'

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
    maintainer='zhangjinrui',
    maintainer_email='1763287396@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_service = py02_service.demo01_service:main',
            'demo02_client = py02_service.demo02_client:main'
        ],
    },
)
