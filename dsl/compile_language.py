from utils import *

def compile_ros_py(ctx: Context, node: Node, emitter=None, file_prefix=None):
    """ Creates a python script capable of running the associated monitor code in ROS """
    data = {}  # data['var1'] = value / data['var2'] = value / emitter << ('file_X.python.jinja', data)
    if emitter is None:
        emitter = Emitter(file_prefix)
        compile_ros_py(ctx, node, emitter)
        return emitter.get_code()
    elif node.type == 'program':
        compile_ros_py(ctx, node.args[0], emitter)  # handle command
        if len(node.args) > 1:
            compile_ros_py(ctx, node.args[1], emitter)  # rest of the program
    elif node.type == 'command':
        compile_ros_py(ctx, node.args[0], emitter) # handle command
    elif node.type == 'declaration':
        #TODO  in what way should i do this?
        var = node.args[0]
        topic = node.args[1]
        msgtype = compile_ros_py(ctx, node.args[2], emitter) # this should return the msgtype
    elif node.type == 'model':
        #TODO
        robot_name = node.args[0]
        topic_type = node.args[1]
        topic_name = node.args[2]
        compile_ros_py(ctx, node.args[3], emitter) # msgtype
    elif node.type == 'msgtype':
        #TODO
        pass
    elif node.type == 'association':
        #TODO
        var_name = node.args[0]
        compile_ros_py(ctx, node.args[1], emitter) # expression
    elif node.type == 'expression':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'pattern_4':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'pattsup':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'paargs':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'operation':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'comparison':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'number':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'operand':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'bool':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'func':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'funcargs':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
    elif node.type == 'temporalvalue':
        #....
        #compile_ros_py(ctx, node.args[0], emitter)
        pass
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