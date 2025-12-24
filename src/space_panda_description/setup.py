from setuptools import setup
from glob import glob
import os

package_name = "space_panda_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],

    data_files=[
        # Package index
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),

        # Package.xml
        ("share/" + package_name,
            ["package.xml"]),

        # URDF
        ("share/" + package_name + "/urdf",
            glob("space_panda_description/urdf/*.xacro")),

        # Launch files
        ("share/" + package_name + "/launch",
            glob("space_panda_description/launch/*.launch.py")),

        # MESHES
        ("share/" + package_name + "/meshes",
            glob("meshes/*.stl")),

        # Controllers YAML
        ("share/" + package_name + "/config",
            glob("space_panda_description/config/*.yaml")),

        # Worlds
        ("share/" + package_name + "/worlds",
            glob("space_panda_description/worlds/*.sdf")),

        # Models 
        ("share/" + package_name + "/models/earth_high_res",
            ["space_panda_description/models/earth_high_res/model.config", "space_panda_description/models/earth_high_res/model.sdf"]),
        ("share/" + package_name + "/models/earth_high_res/materials/scripts",
            glob("space_panda_description/models/earth_high_res/materials/scripts/*")),
        ("share/" + package_name + "/models/earth_high_res/materials/textures",
            glob("space_panda_description/models/earth_high_res/materials/textures/*")),
        ("share/" + package_name + "/models/earth_high_res/resources",
            glob("space_panda_description/models/earth_high_res/resources/*")),
        
        ("share/" + package_name + "/models/sun",
            ["space_panda_description/models/sun/model.config", "space_panda_description/models/sun/model.sdf"]),
        ("share/" + package_name + "/models/sun/materials/scripts",
            glob("space_panda_description/models/sun/materials/scripts/*")),
        ("share/" + package_name + "/models/sun/materials/textures",
            glob("space_panda_description/models/sun/materials/textures/*")),
    ],

    install_requires=["setuptools"],
    zip_safe=True,

    maintainer="AnishNavalgund",
    maintainer_email="anish.navalgund@space-ts.com",
    description="Space-mounted Franka FR3 description",
    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            "pick_place_demo = space_panda_description.pick_place_demo:main",
            "orbital_controller = space_panda_description.orbital_controller:main",
            "arm_demo = space_panda_description.arm_demo:main",
            "camera_controller = space_panda_description.camera_controller:main",
            "camera_gui = space_panda_description.camera_gui_controller:main",
            "camera_opencv_view = space_panda_description.camera_opencv_viewer:main",
        ],
    },
)
