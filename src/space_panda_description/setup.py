from setuptools import setup

package_name = "space_panda_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", ["space_panda_description/urdf/space_panda.urdf.xacro"]),
        ("share/" + package_name + "/launch", ["space_panda_description/launch/view_space_panda.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anish",
    maintainer_email="anishk.navalgund@gmail.com",
    description="Space-mounted Franka Panda (FR3) description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
