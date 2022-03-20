from utils import *

def compile_py(node: Node, ctx=None, file_prefix=None, filepath=None, from_command=False, _property=''):
    """ Creates the template of a python script capable of running the associated monitor code in ROS """
    if ctx is None:
        ctx = CompileContext(file_prefix, filepath)
        compile_py(node, ctx)
        return ctx.get_code()
    elif not type(node) is Node:
        return node  # The node itself is the expected value
    elif node.type == 'program':
        compile_py(node.args[0], ctx)
        if len(node.args) > 1:
            compile_py(node.args[1], ctx)
    elif node.type == 'command':
        compile_py(node.args[0], ctx, from_command=True)
    elif node.type == 'declaration':
        msg_type = compile_py(node.args[2], ctx)
        ctx.add_subscriber(node.args[1], msg_type[0], ctx.get_library(msg_type[0]), node.args[0])
        extract = node.args[0] + '_msg.' + '.'.join(msg_type[1:])
        ctx.add_var(node.args[0], extract)
    elif node.type == 'model':
        modelargs = compile_py(node.args[1], ctx)
        for dic in modelargs:
            msg_type = dic.get('msgtype')
            sub_name = dic.get('var')
            ctx.add_subscriber(dic.get('topic_name'), msg_type[0], ctx.get_library(msg_type[0]), sub_name)
            extract = sub_name + '_msg.' + '.'.join(msg_type[1:])
            ctx.add_var(dic.get('var'), extract)
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
        comparisons = None
        paargs_1 = compile_py(node.args[2], ctx, _property=node.args[0])
        if len(node.args) < 4:
            comparisons = paargs_1
        elif len(node.args) < 6:
            paargs_2 = compile_py(node.args[3], ctx, _property=node.args[0])
            if node.args[0] == 'after':
                comparisons = [paargs_1[0] + ' and ' + paargs_2[0]]
            else:
                comparisons = paargs_1 + paargs_2
        else:
            paargs_2 = compile_py(node.args[3], ctx, _property=node.args[0])
            paargs_3 = compile_py(node.args[4], ctx, _property=node.args[0])
            comparisons = paargs_1 + paargs_2 + paargs_3
        if from_command:
            ctx.add_property('pattern_var_' + str(ctx.get_prop_counter()), comparisons, node.args[1])
        return comparisons
    elif node.type == 'paargs':
        return compile_py(node.args[0], ctx, _property=_property)
    elif node.type == 'pattern_multi':
        op = compile_py(node.args[2], ctx)
        left = compile_py(node.args[0], ctx, _property=_property)
        right = compile_py(node.args[1], ctx, _property=_property)
        if len(node.args) > 3:
            pattsup = compile_py(node.args[3], ctx, _property=_property)
            return [left[0] + ' ' + op + ' ' + right[0] + pattsup[0]]
        else:
            return [left[0] + ' ' + op + ' ' + right[0]]
    elif node.type == 'pattsup':
        op = compile_py(node.args[0], ctx)
        if len(node.args) > 2:
            return ' ' + op + ' ' + compile_py(node.args[1], ctx, _property=_property)[0] + compile_py(node.args[2], ctx, _property=_property)[0]
        else:
            return [' ' + op + ' ' + compile_py(node.args[1], ctx, _property=_property)[0]]
    elif node.type == 'comparison':
        left = compile_py(node.args[0], ctx)
        right = compile_py(node.args[2], ctx)
        op = compile_py(node.args[1], ctx)
        #if len(node.args) > 3:  # has error margin
            #TODO see how to handle this
        return [prop_prefix(_property) + str(left) + str(op) + str(right)]
    elif node.type == 'expression':
        return compile_py(node.args[0], ctx)
    elif node.type == 'operand':
        return compile_py(node.args[0], ctx)
    elif node.type == 'func':
        ctx.add_sim_subscriber()
        if len(node.args) > 2:
            args = compile_py(node.args[2], ctx)
        else:
            args = []
        return sim_funcs(node.args[0], node.args[1], args, ctx)
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]