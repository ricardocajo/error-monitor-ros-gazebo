from utils import *

def verify(ctx: Context, node: Node):
    """ Verifies the syntax of the language"""
    #ctx.add_subscriber('/topic/example', 'Odometry', 'nav_msgs', 'robot1_odom_sub') # use this when decl topic
    ctx.add_sim_subscriber() # use this when one of my funcs is used
    #ctx.add_subscriber('/topic/example_2', 'Vector3', 'geometry_msgs', 'robot1_vel_sub')
    #ctx.add_var('robot1_odom_var', 'robot1', 'robot1_odom_sub', 'position')
    ctx.add_var('robot1_posx_var_sim', 'turtlebot3_burger', None, 'pose', 'position.x')
    ctx.add_var('robot1_posy_var_sim', 'turtlebot3_burger', None, 'pose', 'position.y')
    ctx.add_var('robot1_vel_var_sim', 'turtlebot3_burger', None, 'twist', 'linear.x')
    ctx.add_property('always')
    ctx.add_property('never')
    return True