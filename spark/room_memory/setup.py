from setuptools import find_packages, setup

setup(
    name="room_memory",
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*", "acceptance", "acceptance.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/room_memory"]),
        ("share/room_memory", ["package.xml"]),
        ("share/room_memory/launch", ["launch/room_memory.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Trevor Conger",
    maintainer_email="taconger@students.unwsp.edu",
    description="LabCommand room memory node for the DGX Spark.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "room_memory_node = room_memory.node:main",
            "room_memory_offline = room_memory.offline:main",
        ],
    },
)
