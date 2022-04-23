from __future__ import annotations

import os
import subprocess

from distutils.dir_util import copy_tree

# (["command", ...], "rosbag_name", "lauch_file")
tests = [
    (
        ["_timeout_ = 60", "always turtlebot3_burger.position.x >= -1"],
        "rosbag1.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "never turtlebot3_burger.position.x < -1"],
        "rosbag2.bag",
        "stop.launch",
    ),
]
ros_workspace_dir = "/home/rcordeiro/ros_workspace"
ros_workspace_tests_dir = "/home/rcordeiro/ros_workspace/src/test_pkg/src"

copy_tree(
    "/home/rcordeiro/sim_monitor_compiler/unit_tests/rosbags",
    f"{ros_workspace_tests_dir}/bagfiles",
)
copy_tree(
    "/home/rcordeiro/sim_monitor_compiler/unit_tests/launches",
    f"{ros_workspace_tests_dir}/launches",
)
# output
for test in tests:
    with open("test_.txt", "w") as f_out:
        f_out.write("\n".join(test[0]))
    os.system(
        f"cd .. ; python language.py unit_tests/test_.txt {ros_workspace_tests_dir}",
    )
    subprocess.Popen(
        f"cd {ros_workspace_tests_dir} ; rosrun test_pkg test_.py",
    )  # need to catch output
    # os.system(
    #    "cd " + ros_workspace_tests_dir + "/bagfiles ; rosbag play --clock " + test[1]
    # )
    # os.system(
    #    "cd "
    #    + ros_workspace_dir
    #    + " ; roslaunch test_pkg "
    #    + test[2]
    #    + " use_sim_time:=true"
    # )
    # save the output of the test in a var
    os.remove("/home/rcordeiro/sim_monitor_compiler/unit_tests/test_.txt")
    os.remove(f"{ros_workspace_tests_dir}/test_.py")
# send results to mattermost
