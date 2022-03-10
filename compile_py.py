from utils import *

def compile_py(node: Node, ctx=None, file_prefix=None, filepath=None, from_command=None, add=''):
    """ Creates the template of a python script capable of running the associated monitor code in ROS """
    if ctx is None:
        ctx = CompileContext(file_prefix, filepath)
        compile_py(node, ctx)
        return ctx.get_code()
    elif not type(node) is Node:
        return node  # The node itself is the expected value
    elif node.type == 'program':
        compile_py(node.args[0], ctx)  # command
        if len(node.args) > 1:
            compile_py(node.args[1], ctx)
    elif node.type == 'command':
        compile_py(node.args[0], ctx, from_command=True)
    elif node.type == 'declaration':
        msg_type = compile_py(node.args[2], ctx)
        ctx.add_subscriber(node.args[1], msg_type[0], ctx.get_library(msg_type[0]), node.args[0])
        ctx.add_var(node.args[0], None, node.args[0], '.'.join(msg_type[1:]), None)
    elif node.type == 'model':
        modelargs = compile_py(node.args[1], ctx)
        for dic in modelargs:
            msg_type = dic.get('msgtype')
            sub_name = dic.get('var')
            ctx.add_subscriber(dic.get('topic_name'), msg_type[0], ctx.get_library(msg_type[0]), sub_name)
            ctx.add_var(dic.get('var'), None, sub_name, '.'.join(msg_type[1:]), None)
    elif node.type == 'modelargs':
        return_dic = [{'var': node.args[0], 'topic_name': node.args[1], 'msgtype': compile_py(node.args[2], ctx)}]
        if len(node.args) > 3:
            return return_dic + compile_py(node.args[3], ctx)
        return return_dic
    elif node.type == 'msgtype':  # returns a list with msgtype and args
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'association':
        ctx.add_assoc(node.args[0], compile_py(node.args[1]))
    elif node.type == 'property':
        if len(node.args) < 3:
            _list = compile_py(node.args[1], ctx, add=add)
            ctx.add_property(node.args[0], _list, from_command)
            return [{'type': 'pattern', 'list': ctx.get_prop_func(node.args[0]), 'add': add}]
        elif len(node.args) < 5:
            left = compile_py(node.args[1], ctx, add=add)
            right = compile_py(node.args[2], ctx, add=add)
            _list = {'list1': left, 'list2': right}
            ctx.add_property(node.args[0], _list, from_command)
            return [{'type': 'pattern', 'list': ctx.get_prop_func(node.args[0]), 'add': add}]
        else:
            _list1 = compile_py(node.args[1], ctx)
            _list2 = compile_py(node.args[2], ctx)
            _list3 = compile_py(node.args[3], ctx)
    elif node.type == 'paargs':
        if add != '':
            return compile_py(node.args[0], ctx, from_command=False, add=add)
        else:
            return compile_py(node.args[0], ctx, from_command=False)
    elif node.type == 'pattern_multi':
        op = compile_py(node.args[2], ctx) + ' '
        left = compile_py(node.args[0], ctx)
        if len(node.args) > 3:
            right = compile_py(node.args[1], ctx, add=op)
            pattsup = compile_py(node.args[3], ctx)
            return left + right + pattsup
        else:
            right = compile_py(node.args[1], ctx, add=op)
            return left + right
    elif node.type == 'pattsup':
        op = compile_py(node.args[0], ctx) + ' '
        if len(node.args) > 2:
            return compile_py(node.args[1], ctx, add=op) + compile_py(node.args[2], ctx)
        else:
            return compile_py(node.args[1], ctx, add=op)
    elif node.type == 'comparison':
        left = compile_py(node.args[0], ctx)
        right = compile_py(node.args[2], ctx)
        op = compile_py(node.args[1], ctx)
        #if len(node.args) > 3:  # has error margin
            #TODO see how to handle this
        return [{'type': 'comparison', 'var1': left, 'op_bin': op, 'var2': right, 'add': add}]
    elif node.type == 'expression':
        return compile_py(node.args[0], ctx)
    elif node.type == 'operand':
        return compile_py(node.args[0], ctx)
    elif node.type == 'func':
        ctx.add_sim_subscriber()
        func = node.args[1]
        if len(node.args) > 2:
            args = compile_py(node.args[2], ctx)
        else:
            args = []
        return sim_funcs(node.args[0], func, args, ctx)
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]