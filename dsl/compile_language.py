from utils import *

def compile_ros_py(ctx: Context, file_prefix: str):
    """ Creates a python script capable of running the associated monitor code in ROS """
    emitter = Emitter(file_prefix)
    emitter << ('subscribers.jinja', ctx.get_subscribers())
    return emitter.get_code()