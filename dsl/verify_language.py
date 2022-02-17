from utils import *

def verify(ctx: Context, node: Node):
    """ Verifies the syntax of the language"""
    ctx.add_subscriber('/topic/example', 'Vector3', 'callback_func')
    ctx.add_subscriber('/topic/example_2', 'Vector3', 'callback_func_2')
    return True