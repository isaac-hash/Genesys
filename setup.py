from setuptools import setup, find_packages

setup(
    name="genesys_framework_cli",
    version="0.1.5",
    author="Chukwudulue Isaac",
    author_email="zikkychukwudulue@gmail.com",
    description="A developer-friendly, opinionated framework for ROS 2.",
    long_description=open("README.md", "r", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    python_requires=">=3.8",
    packages=find_packages(where="."),
    # This flag tells setuptools to include files specified in MANIFEST.in or package_data
    include_package_data=True,
    # This explicitly lists non-Python files to be included in the package
    package_data={
        # Include all files within the 'genesys-macros' directory inside the 'genesys_cli' package
        'genesys_cli': ['genesys-macros/**'],
    },
    install_requires=[
        "click",
        "Jinja2",
        "pydantic",
        "PyYAML",
        'importlib-resources; python_version < "3.9"',
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent"
    ],
    entry_points={
        "console_scripts": [
            "genesys=genesys_cli.main:cli",
        ],
    },
    url="https://github.com/isaac-hash/Genesys.git",
    project_urls={
        "Homepage": "https://github.com/isaac-hash/Genesys.git",
        "Issues": "https://github.com/isaac-hash/Genesys/issues"
    },
)
