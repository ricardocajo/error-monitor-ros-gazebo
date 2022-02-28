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
        ctx.add_subscriber(node.args[1], msg_type[0], ctx.get_library(msg_type[0]), node.args[0] + '_sub')
        ctx.add_var(node.args[0], None, node.args[0] + '_sub', '.'.join(msg_type[1:]), None)
    elif node.type == 'model':
        modelargs = compile_py(node.args[1], ctx)
        for dic in modelargs:
            msg_type = dic.get('msgtype')
            sub_name = dic.get('var') + '_sub'
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
        return {'comp1_var1': left, 'comp1_var2': right, 'bin_op1': op}
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
            args = None
        return sim_funcs(node.args[0], func, args, ctx)
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    else:
        return node

'''
# always property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if not {{ prop.comp1_var1 }} {{ prop.op_bin }} {{ prop.comp1_var2 }}: and/or not x
            print("my_error_message")
            {{ prop.property }}_pattern_var{{ index }} = False
            
# always property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        do_after()
        if not global_after_var: and/or not x
            print("my_error_message")
            {{ prop.property }}_pattern_var{{ index }} = False

# always property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if not given_var: and/or not x
            print("my_error_message")
            {{ prop.property }}_pattern_var{{ index }} = False
            
            
            
# never property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if {{ prop.comp1_var1 }} {{ prop.op_bin }} {{ prop.comp1_var2 }}: and/or x
            print("my_error_message")
            {{ prop.property }}_pattern_var{{ index }} = False
            
            
  
# after property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if comp1_var1 opbin comp1_var2: and/or x
            if comp2_var1 opbin comp2_var2: and/or x
                print("my error message after")
                {{ prop.property }}_pattern_var{{ loop.index }} = False
                
                
                
# until property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if not comp1_var1 opbin comp1_var2: and/or not x
            if comp2_var1 opbin comp2_var2: and/or x
                print("my error message after")
                {{ prop.property }}_pattern_var{{ loop.index }} = False
                
                
                
# after_until property ...
    if {{ prop.property }}_pattern_var{{ index }}:
        if comp1_var1 opbin comp1_var2 and not comp2_var1 opbin comp2_var2: and/or x | and/or not x
            if comp3_var1 opbin comp3_var2: and/or x
                print("my error message after")
                {{ prop.property }}_pattern_var{{ loop.index }} = False
                
                
                
# eventually property ...
    if {{ prop.property }}_pattern_var{{ index }}:

                {{ prop.property }}_pattern_var{{ loop.index }} = False
'''