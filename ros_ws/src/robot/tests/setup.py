from setuptools import setup, find_packages

setup(
    name="robot_tests", 
    version="0.1.0",
    description="A testing package for robot system interaction",
    author="Tyler Harp",
    author_email="tharp@andrew.cmu.edu",
    packages=find_packages(),  # Automatically find and include all packages in the directory
    install_requires=[
        # List your package's dependencies here
    ],
    tests_require=[
        "pytest", "unittest"  # Include pytest for running tests
    ],
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    #     "License :: OSI Approved :: MIT License",
    #     "Operating System :: OS Independent",
    #     "Development Status :: 3 - Alpha", 
    #     "Intended Audience :: Developers",
    #     "Topic :: Software Development :: Testing",
    # ],
    python_requires=">=3.6",
    entry_points={
        "console_scripts": [
            "mypackage=mypackage.__main__:main",  
        ],
    },
)