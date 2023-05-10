import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'mr_joint_space_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),        
    	('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),        
    	('share/' + package_name+'', glob('urdf/*')),            
    	(os.path.join('share', package_name), glob('urdf/*.rviz'))  
	
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sung',
    maintainer_email='tjdalsckd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = publisher.state_publisher:main'        
        ],
    },
)
