""" Tests to be ran in unit_test.py """
# (["command", ...], "rosbag_name", "lauch_file")
from __future__ import annotations

tests = [
    (
        ["_timeout_ = 60", "always turtlebot3_burger.position.x >= -1"],
        "rosbag2.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "always turtlebot3_burger.position.x >= -1"],
        "rosbag2.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "never turtlebot3_burger.position.x < -1"],
        "rosbag2.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "never turtlebot3_burger.position.x < -1"],
        "rosbag2.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "never turtlebot3_burger.velocity > 0"],
        "rosbag1.bag",
        "stop.launch",
    ),
    (
        ["_timeout_ = 60", "never turtlebot3_burger.velocity > 0"],
        "rosbag2.bag",
        "stop.launch",
    ),
]
