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
    elif node.type == 'msgtype':  # returns a list with msgtype and args
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'property': # TODO i might be able to do this without checking for the properties names
        vars_dic = compile_py(node.args[1], ctx)
        ctx.add_property(node.args[0], vars_dic.get('comp_var1'), vars_dic.get('comp_var2'), vars_dic.get('bin_op'))
    elif node.type == 'paargs':
        return compile_py(node.args[0], ctx)
    elif node.type == 'comparison':
        left = compile_py(node.args[0], ctx)
        right = compile_py(node.args[2], ctx)
        op = compile_py(node.args[1], ctx)
        if len(node.args) > 3:  # has error margin
            #TODO see how to handle this
            pass
        return {'comp_var1': left, 'comp_var2': right, 'bin_op': op}
    elif node.type == 'opbin':
        return node.args[0]
    elif node.type == 'expression':
        return compile_py(node.args[0], ctx)
    elif node.type == 'operand':
        return compile_py(node.args[0], ctx)
    elif node.type == 'number':
        return node.args[0]
    elif node.type == 'bool':
        return node.args[0]
    elif node.type == 'func':
        ctx.add_sim_subscriber()
        func = node.args[1]
        if len(node.args) > 2:
            args = compile_py(node.args[2], ctx)
        else:
            args = None
        return sim_funcs(node.args[0], func, args, ctx)
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    else:
        return node