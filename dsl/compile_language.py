from utils import *
import os
from pathlib import Path

def compile_ros_py(ctx: Context, node: Node, emitter=None, file_prefix=None):
    """ Creates a python script capable of running the associated monitor code in ROS """
    if emitter is None:
        emitter = Emitter()
        emitter << "#! /usr/bin/env python"
        emitter << "import rospy"
        emitter << "from ros_func_lib.predicates_func import *"
        emitter << f"rospy.init_node('{file_prefix}')"
        emitter << "print(\"Monitoring...\")"
        compile_ros_py(ctx, node, emitter)
    elif node.type == 'opargs':
        #....
        compile_ros_py(ctx, node.args[0], emitter)
    else:
        print("Node type doesn't exist")
    
    #I need to see how to handle getting messages synchronized to use in the functions

    #emitter << "msg = get_topic_msg_modelStates()"
    #emitter << "x = position_x(\"turtlebot3_burger\", msg)"
    #emitter << "y = position_y(\"turtlebot3_burger\", msg)"
    #How to check every frame of the simulation for the always keyword
    #emitter << "while True:"
    #emitter << "    if x < 0.0:"
    #emitter << "        print(\"Rule broken\")"
    #emitter << "        break"
    #emitter << "    if y < 0.0:"
    #emitter << "        print(\"Rule broken\")"
    #emitter << "        break"

    return emitter.get_code()