from setuptools import find_packages, setup

package_name = 'py01_topic'

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
    maintainer='ubuntu',
    maintainer_email='1763287396@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_talker_py = py01_topic.demo01_talker_py:main',
            'demo02_listener_py = py01_topic.demo02_listener_py:main',
            'demo03_talker_stu = py01_topic.demo03_talker_stu:main',
            'demo04_listener_stu= py01_topic.demo04_listener_stu:main'
        ],
    },
)
