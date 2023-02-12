from setuptools import setup

PACKAGE_NAME = "ron_basic_viz"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="R. Nick Vandemark",
    author_email="rnvandemark@gmail.com",
    maintainer="R. Nick Vandemark",
    maintainer_email="rnvandemark@gmail.com",
    keywords=["ROS2"],
    description="A basic visualization for a RON system.",
    license="Apache License, Version 2.0",
    entry_points={
        "console_scripts": [
            "viz = " + PACKAGE_NAME + ".ron_viz:main",
        ],
    },
)
