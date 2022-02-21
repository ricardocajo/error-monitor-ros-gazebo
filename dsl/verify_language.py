from utils import *

def verify(ctx: Context, node: Node):
    """ Verifies the syntax of the language"""
    ctx.add_subscriber('/topic/example', 'Odometry', 'nav_msgs', 'robot1_odom_sub') # use this when decl topic
    ctx.add_sim_subscriber() # use this when one of my funcs is used
    ctx.add_subscriber('/topic/example_2', 'Vector3', 'geometry_msgs', 'robot1_vel_sub')
    ctx.add_var('robot1_odom_var', 'robot1', 'robot1_odom_sub', 'position')
    ctx.add_var('robot1_pos_var_sim', 'robot1', None, 'pose', 'x')
    #ctx.add_property()
    return True