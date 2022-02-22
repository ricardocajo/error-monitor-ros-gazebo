from utils import *

def compile_py(ctx: CompileContext, file_prefix: str, node: Node):
    """ Creates a python script capable of running the associated monitor code in ROS """
    if node.type == 'program':
        #.... do something
        pass
    elif node.type == 'command':
        #.... do something
        pass

    return None

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
    
    file_loader = FileSystemLoader('templates')
    env = Environment(loader=file_loader,extensions=['jinja2.ext.do'])
    template = env.get_template('program.jinja')
    return template.render(file_prefix=file_prefix, sim_sub=ctx.get_sim_subscriber(), 
                           subscribers=ctx.get_subscribers(), var_list=ctx.get_vars(),
                           properties=ctx.get_properties())
'''