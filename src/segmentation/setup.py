from setuptools import setup

package_name = 'segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['share/cgnet.ts', 'package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naza',
    maintainer_email='nozolo90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation = segmentation.segmentation:main',
            'visualize = segmentation.visualize:main'
        ],
    },
)
