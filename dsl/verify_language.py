from utils import *

def verify(ctx: Context, node: Node):
    """ Verifies the syntax of the language"""
    ctx.add_subscriber('/topic/example', 'Odometry', 'nav_msgs')
    ctx.add_subscriber('/topic/example_2', 'Vector3', 'geometry_msgs')
    ctx.add_property()
    return True