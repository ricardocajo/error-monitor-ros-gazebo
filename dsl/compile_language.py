from utils import Context
import os
from pathlib import Path

def compile_ros_py(ctx:Context, node):
    """ Creates a ros package capable of running the associated monitor code"""
    #create ros package? or create file in a given package
    home = str(Path.home())
    ros_workspace_directory = os.path.join(home, "ros_workspace/src")
    test_directory = "teste"
    path = os.path.join(ros_workspace_directory, test_directory)
    Path(path).mkdir(parents=True, exist_ok=True)

    filename = "teste.py"
    filepath = os.path.join(path,filename)
    f = open(filepath, "w")

    return True