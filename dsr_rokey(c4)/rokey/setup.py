from setuptools import find_packages, setup

package_name = "rokey"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juwan",
    maintainer_email="dlacksdn352@gmail.com",
    description="ROKEY BOOT CAMP Package",
    license="Apache 2.0 License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_turn=rokey.basic.test_turn:main",
            "test_stack=rokey.basic.test_stack:main",
            "test_gear=rokey.basic.test_gear:main",
            "force_control = rokey.basic.force_control:main",
            "jog_example = rokey.basic.jog_example:main",
            "jog = rokey.basic.jog_complete:main",
            "move_periodic = rokey.basic.move_periodic:main",
            "getting_position = rokey.basic.getting_position:main",
            "simple_move=rokey.basic.move:main",
            "grip=rokey.basic.grip:main",
            "get_current_pos=rokey.basic.get_current_pos:main",
            "domino = rokey.basic.domino:main",
            "cup = rokey.basic.cup:main", 
            "cup_stack = rokey.basic.cup_stack:main",
            "gear = rokey.basic.gear:main",
            
        ],
    },
)
