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
        ctx.add_subscriber(node.args[1], compile_py(node.args[2]), ctx.get_library(), node.args[0] + '_sub')
        pass

'''
#ctx.add_subscriber('/topic/example', 'Odometry', 'nav_msgs', 'robot1_odom_sub') # use this when decl topic
    ctx.add_sim_subscriber() # use this when one of my funcs is used
    #ctx.add_subscriber('/topic/example_2', 'Vector3', 'geometry_msgs', 'robot1_vel_sub')
    #ctx.add_var('robot1_odom_var', 'robot1', 'robot1_odom_sub', 'position')
    ctx.add_var('robot1_posx_var_sim', 'turtlebot3_burger', None, 'pose', 'position.x')
    ctx.add_var('robot1_posy_var_sim', 'turtlebot3_burger', None, 'pose', 'position.y')
    ctx.add_var('robot1_vel_var_sim', 'turtlebot3_burger', None, 'twist', 'linear.x')
    ctx.add_property('always')
    ctx.add_property('never')
'''