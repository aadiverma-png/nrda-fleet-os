from setuptools import find_packages, setup

setup(
    name="nrda_core",
    version="1.0.0",
    description="NRDA-FA: decentralised edge-AI AMR fleet coordination (ROS-free core)",
    packages=find_packages(include=["visison", "nrda_fleet", "nrda_fleet.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/nrda_core"]),
        ("share/nrda_core", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    python_requires=">=3.10",
)
