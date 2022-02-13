from utils import *

def compile_ros_py(ctx: Context, node: Node, emitter=None, file_prefix=None):
    """ Creates a python script capable of running the associated monitor code in ROS """
    data = {}  # data['var1'] = value / data['var2'] = value / emitter << ('file_X.python.jinja', data)
    print(node)
    if emitter is None:
        emitter = Emitter(file_prefix)
        compile_ros_py(ctx, node, emitter)
        return emitter.get_code()
    elif node.type == 'program':
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