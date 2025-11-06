from setuptools import setup, find_packages

setup(
    name='genesys-framework-cli',
    version='0.1.5',
    # find_packages() will automatically discover `genesys_cli` and `genesys`
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        'click',
        'Jinja2',
        # rclpy and other ROS packages are expected to be in the sourced environment
    ],
    entry_points={
        'console_scripts': [
            # This creates the `genesys` command and points it to your main function
            'genesys = genesys_cli.main:cli',
        ],
    },
    author='Genesys Developer',
    author_email='dev@genesys.ros',
    description='A developer-friendly, opinionated framework for ROS 2.',
    license='Apache 2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['org.txt']),
        ('share/genesys/cmake', ['genesys/genesys.cmake']),
        ('include/genesys', ['genesys/include/genesys/macros.hpp']),
    ],
)