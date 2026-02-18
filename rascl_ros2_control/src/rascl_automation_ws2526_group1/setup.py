from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rascl_automation_ws2526_group1"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "automation_tsk1 = rascl_automation_ws2526_group1.automation_tsk1:main",
            "automation_tsk2 = rascl_automation_ws2526_group1.automation_tsk2:main",
            "pick_and_place_mc = rascl_automation_ws2526_group1.motion_controller_main:main",
            "rtb_test = rascl_automation_ws2526_group1.rtb_test:main",
            "goal_pose_gui = rascl_automation_ws2526_group1.testMsgBuff:main",
            "pub_orig_dest = rascl_automation_ws2526_group1.pub_orig_dest:main",
            "one_position = rascl_automation_ws2526_group1.one_position_test:main",
        ],
    },
)
