from utils import *

def compile_py(node: Node, ctx=None, file_prefix=None, filepath=None):
    """ Creates the template of a python script capable of running the associated monitor code in ROS """
    if ctx is None:
        ctx = CompileContext(file_prefix, filepath)
        compile_py(node, ctx)
        return ctx.get_code()
    elif node.type == 'program':
        compile_py(node.args[0], ctx)  # command
        if len(node.args) > 1:
            compile_py(node.args[1], ctx)
    elif node.type == 'command':
        compile_py(node.args[0], ctx)
    elif node.type == 'declaration':
        msg_type = compile_py(node.args[2], ctx)
        ctx.add_subscriber(node.args[1], '.'.join(msg_type), ctx.get_library(msg_type[0]), node.args[0] + '_sub')
        pass
    elif node.type == 'msgtype':  # returns a list with msgtype and args
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'property': # TODO i might be able to do this without checking for the properties names
        if node.args[0] == 'always': # TODO when i compile the args i might need to get the info of vars etc.. 
            compile_py(node.args[1], ctx)
            ctx.add_property('always')
        if node.args[0] == 'never':
            #do never
            pass
        #etc....
    elif node.type == 'paargs':
        compile_py(node.args[0])
    elif node.type == 'comparison':
        left = compile_py(node.args[0])
        right = compile_py(node.args[2])
        op = compile_py(node.args[1])
        if len(node.args) > 3:  # has error margin
            #TODO see how to handle this
            pass
        #TODO  see what to do
    elif node.type == 'opbin':
        return node.args[0]
    elif node.type == 'expression':
        compile_py(node.args[0], ctx)
    elif node.type == 'operand':
        compile_py(node.args[0])
    elif node.type == 'number':
        return node.args[0]
    elif node.type == 'bool':
        return node.args[0]
    elif node.type == 'func':
        ctx.add_sim_subscriber()
        #TODO the var value will be the corresponding code of acessing the modelstates topic
    else:
        return node

'''
    ctx.add_var('robot1_odom_var', 'robot1', 'robot1_odom_sub', 'position')
    ctx.add_var('robot1_posx_var_sim', 'turtlebot3_burger', None, 'pose', 'position.x')
    ctx.add_var('robot1_posy_var_sim', 'turtlebot3_burger', None, 'pose', 'position.y')
    ctx.add_var('robot1_vel_var_sim', 'turtlebot3_burger', None, 'twist', 'linear.x')
    ctx.add_property('always')
    ctx.add_property('never')
'''