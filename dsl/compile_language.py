from utils import *

def compile_ros_py(ctx: Context, file_prefix: str):
    """ Creates a python script capable of running the associated monitor code in ROS """
    emitter = Emitter(file_prefix)
    print(ctx.get_subscribers())
    emitter << ('subscribers.jinja', ctx.get_subscribers())
    return emitter.get_code()
    

    
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