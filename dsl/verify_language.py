from utils import *

def verify(ctx: Context, node: Node):
    """ Verifies the syntax of the language"""
    ctx.add_subscriber('/topic/example', 'Odometry', 'nav_msgs') # use this when decl topic
    ctx.add_sim_subscriber() # use this when one of my funcs is used
    ctx.add_subscriber('/topic/example_2', 'Vector3', 'geometry_msgs')
    #ctx.add_property()
    return True