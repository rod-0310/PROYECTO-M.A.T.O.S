from setuptools import find_packages, setup

package_name = 'kinect_compressor'

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
    maintainer='weimar',
    maintainer_email='rufo.huallpara@ucb.edu.bo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_image = kinect_compressor.kinect_image_republisher:main',
            'depth_to_scan = kinect_compressor.launch_depth_to_scan:generate_launch_description',
            'fake_odom = kinect_compressor.fake_odom:main',
            'modo_giro = kinect_compressor.modo_giro:main',
            'pid_controller = kinect_compressor.pidyolo:main',
            
        ],
    },
)
