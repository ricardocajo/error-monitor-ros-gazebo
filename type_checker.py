from utils import *

def type_checker(ctx: TypeCheckerContext, node: Node, filepath: str):
    """ Verifies the syntax of the language"""
    #TODO verify if filepath is in rosworkspace
    #TODO make sure error margin is only in  == and != not in > etc.
    if node.type == 'program':
        #.... do something
        pass
    elif node.type == 'command':
        #.... do something
        pass
    #TODO 
    return None