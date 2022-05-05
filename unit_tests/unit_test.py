from __future__ import annotations

import json
import os
import signal
import subprocess
from datetime import date
from subprocess import PIPE
from subprocess import Popen
from subprocess import STDOUT

import requests
from distutils.dir_util import copy_tree
from tests import *

# This is a percentage (%)
warning_treshold = 15

ros_workspace_dir = "/ros_ws"
ros_workspace_tests_dir = "/ros_ws/src/test_pkg/src"

copy_tree(
    "/sim_monitor_compiler/unit_tests/rosbags",
    f"{ros_workspace_tests_dir}/bagfiles",
)
copy_tree(
    "/sim_monitor_compiler/unit_tests/launches",
    f"{ros_workspace_tests_dir}/launches",
)
results = [
    f"### **Test results for {date.today()}**",
    "| Test Number  | Output  | Result |",
    "| :------------ |:---------------:|:-----:|",
]
loop_counter, error_counter = 0, 0
for test in tests:
    with open("/sim_monitor_compiler/unit_tests/test_.txt", "w") as f_out:
        f_out.write("\n".join(test[0]))
    os.system(
        f"cd sim_monitor_compiler && python3.8 language.py unit_tests/test_.txt {ros_workspace_tests_dir}",
    )
    p_test = subprocess.Popen(
        f"cd {ros_workspace_dir} && rosrun test_pkg test_.py",
        shell=True,
        stdin=PIPE,
        stdout=PIPE,
        stderr=STDOUT,
    )
    p_bagfile = subprocess.Popen(
        f"cd {ros_workspace_tests_dir}/bagfiles && rosbag play --clock {test[1]}",
        shell=True,
        stdin=PIPE,
        stdout=PIPE,
        stderr=STDOUT,
        preexec_fn=os.setsid,
    )
    p_main = subprocess.Popen(
        f"cd {ros_workspace_dir} && export TURTLEBOT3_MODEL=burger && roslaunch test_pkg {test[2]} use_sim_time:=true",
        shell=True,
        stdin=PIPE,
        stdout=PIPE,
        stderr=STDOUT,
        preexec_fn=os.setsid,
    )

    stdout_test, nothing = p_test.communicate()
    os.killpg(os.getpgid(p_bagfile.pid), signal.SIGTERM)
    os.killpg(os.getpgid(p_main.pid), signal.SIGTERM)

    output = (
        "\n".join(stdout_test.replace(b"\x1b", b"").decode("utf-8").split("\n")[1:])
        .replace("\n", "\t\t")
        .replace("[34m", "")
        .replace("[0m", "")
        .replace("[31m", "")
    )
    emote = ":exclamation:" if output[:5] == "Error" else ":white_check_mark:"
    loop_counter += 1
    error_counter += 1 if emote == ":exclamation:" else 0
    results.append(f"| Test {loop_counter} | {output}| {emote} |")

    os.remove("sim_monitor_compiler/unit_tests/test_.txt")
    os.remove(f"{ros_workspace_tests_dir}/test_.py")

    p_bagfile.wait()
    p_main.wait()

test_error_value = round(100 * error_counter / loop_counter, 2)
test_error_result = (
    ":warning:" if test_error_value > warning_treshold else ":white_check_mark:"
)

results.append(
    f"#### **The test ended with an {str(100 - test_error_value)}% sucess rate. {test_error_result}**",
)

requests.post(
    "https://chat.lasige.di.fc.ul.pt/hooks/xnzswnyyn3nt7mn3jwucezqzby",
    headers={"Content-Type": "application/json"},
    data=json.dumps({"text": "\n".join(results)}),
)
