from setuptools import setup

package_name = 'pkg_d_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test User',
    maintainer_email='test@example.com',
    description='Python package D for parallel build testing',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
