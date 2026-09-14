from setuptools import find_packages, setup


package_name = "module_b_navigation"


setup(
    name=package_name,
    version="0.0.1",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/"
                + package_name
            ],
        ),
        (
            "share/" + package_name,
            [
                "package.xml"
            ],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="wowa",

    maintainer_email=(
        "wowa@example.com"
    ),

    description=(
        "Navigation solution "
        "for CHVT 2026 Module B"
    ),

    license="MIT",

    tests_require=[
        "pytest"
    ],

    entry_points={
        "console_scripts": [
            (
                "module_b = "
                "module_b_navigation.main:main"
            ),
        ],
    },
)
