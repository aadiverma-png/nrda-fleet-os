import os
from glob import glob

from setuptools import setup

package = "nrda_fleet_ros"


def tree(src):
    out = []
    for root, _dirs, files in os.walk(src):
        if files:
            out.append((os.path.join("share", package, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package,
    version="1.0.0",
    packages=[package],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package}"]),
        (f"share/{package}", ["package.xml"]),
        (f"share/{package}/launch", glob("launch/*.py")),
        (f"share/{package}/config", glob("config/*")),
        *tree("web"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NRDA",
    description="NRDA-FA edge agents, lidar emulator, safety monitor, fleet viz and web console",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "edge_agent = nrda_fleet_ros.edge_agent_node:main",
            "edge_agents = nrda_fleet_ros.edge_agent_node:main_group",
            "lidar_emulator = nrda_fleet_ros.lidar_emulator_node:main",
            "safety_monitor = nrda_fleet_ros.safety_monitor_node:main",
            "fleet_viz = nrda_fleet_ros.fleet_viz_node:main",
            "fleet_gateway = nrda_fleet_ros.gateway_node:main",
        ],
    },
)
