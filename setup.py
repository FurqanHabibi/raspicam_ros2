from setuptools import setup

package_name = 'raspicam_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'raspicam_ros2',
        'camera_info_manager'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('shared/' + package_name, ['package.xml']),
        ('lib/python3.5/site-packages/config', ['config/raspicam_1280x960.yaml', 'config/raspicam_416x320.yaml'])
    ],
    install_requires=['setuptools'],
    maintainer='Muhammad Furqan Habibi',
    maintainer_email='furqan.habibi1@gmail.com',
    author='Muhammad Furqan Habibi',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Simple raspberry pi camera v2 node for ROS2.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'raspicam_ros2 = raspicam_ros2:main'
        ],
    },
)
