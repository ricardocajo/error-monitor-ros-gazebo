from utils import *

def type_checker(node: Node, ctx=None, filepath=None):
    """ Verifies the syntax of the language"""
    #TODO make sure error margin is only in  == and != not in > etc.
    if ctx is None:
        #TODO verify if filepath is valid and a rosworkspace
        ctx = TypeCheckerContext()
        type_checker(node, ctx)
    #elif not type(node) is Node:
    #    return
    elif node.type == 'program':
        if len(node.args) > 1:
            type_checker(node.args[1], ctx)
        type_checker(node.args[0], ctx)
    elif node.type == 'command':
        type_checker(node.args[0], ctx)
    elif node.type == 'declaration':
        ctx.add_topic_value(node.args[0])
    elif node.type == 'model':
        type_checker(node.args[1])
    elif node.type == 'modelargs':
        if len(node.args) > 3:
            type_checker(node.args[3])
        ctx.add_topic_value(node.args[0])
    elif node.type == 'association':
        ctx.add_assoc(node.args[0])
    elif node.type == 'property':
        if len(node.args) == 1:
            type_checker(node.args[0])
        if len(node.args) == 3:
            type_checker(node.args[2])
        if len(node.args) == 4:
            type_checker(node.args[2])
            type_checker(node.args[3])
        if len(node.args) == 5:
            type_checker(node.args[2])
            type_checker(node.args[3])
            type_checker(node.args[4])
    elif node.type == 'conjunction':
        if len(node.args) == 1:
            type_checker(node.args[0])
        #TODO check if left and right are both comparisons, if not syntactic error
    elif node.type == 'comparison':
        if len(node.args) == 1:
            type_checker(node.args[0])
    """    if len(node.args) == 4:
            pass
    elif node.type == 'multiplication':
        if len(node.args) == 1:
            pass
        return
    elif node.type == 'addition':
        if len(node.args) == 1:
            pass
        return
    elif node.type == 'operand':
        return
    elif node.type == 'func':
        if len(node.args) > 2:
            pass
        return
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            pass
        return
    elif node.type == 'temporalvalue':
        return"""