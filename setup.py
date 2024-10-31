from setuptools import find_packages, setup

package_name = 'ros2_differential_dr'

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
    maintainer='o2d',
    maintainer_email='dmitry.chistov@2lemetry.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_tf = ros2_differential_dr.diff_tf:main'
        ],
    },
)
