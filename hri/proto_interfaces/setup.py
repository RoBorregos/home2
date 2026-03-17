from setuptools import setup, find_packages

setup(
    name="proto_interfaces",
    version="0.1.0",
    author="RoBorregos",
    author_email="roborregosteam@gmail.com",
    description="Protocol Buffer interfaces for HRI microservices",
    long_description=open("README.md").read() if __import__('os').path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "grpcio>=1.70.0",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
    ],
    url="https://github.com/RoBorregos/home2",
)
