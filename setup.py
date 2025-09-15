from setuptools import setup, find_packages

setup(
    name='genesys-framework-cli',
    version='0.1.1',
    # find_packages() will automatically discover `framework_cli` and `framework_core`
    packages=find_packages(),
    install_requires=[
        'click',
        # rclpy and other ROS packages are expected to be in the sourced environment
    ],
    entry_points={
        'console_scripts': [
            # This creates the `genesys` command and points it to your main function
            'genesys = framework_cli.main:cli',
        ],
    },
    author='Genesys Developer',
    author_email='dev@genesys.ros',
    description='A developer-friendly, opinionated framework for ROS 2.',
    license='Apache 2.0',
)