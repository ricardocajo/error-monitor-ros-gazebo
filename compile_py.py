from utils import *
import re

def compile_py(node: Node, ctx=None, file_prefix=None, filepath=None, from_command=False, property_=''):
    """ Creates the template of a python script capable of running the associated monitor code in ROS """
    if ctx is None:
        ctx = CompileContext(file_prefix, filepath)
        compile_py(node, ctx)
        return ctx.get_code()
    elif not type(node) is Node:  # The node itself is the expected value
        if ctx.is_assoc(node):
            return ctx.assoc_info(node)
        return node,node,[],[],False,['','','','']
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
    elif node.type == 'msgtype':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'association':
        ctx.add_assoc(node.args[0], compile_py(node.args[1], ctx))
    elif node.type == 'rate':
        ctx.rate_update(node.args[0])
    elif node.type == 'timeout':
        ctx.timeout_update(node.args[0])
    elif node.type == 'property':
        if len(node.args) == 1:
            return compile_py(node.args[0], ctx, property_=property_)
        if len(node.args) == 3:
            eventually_check = False
            if node.args[0] == 'eventually':
                eventually_check = True
            arg_1 = compile_py(node.args[2], ctx, property_=node.args[0])
            comparisons = arg_1[0], node.args[0]+' '+arg_1[1],arg_1[2],arg_1[3],arg_1[4] or eventually_check,arg_1[5]
        if len(node.args) == 4:
            arg_1 = compile_py(node.args[2], ctx, property_=node.args[0])
            arg_2 = compile_py(node.args[3], ctx)
            if node.args[0] == 'until' and arg_2[4]:
                pass
            if node.args[0] == 'until' and arg_2[4] and arg_2[5] == 'and':
                pass
            if node.args[0] == 'until' and arg_2[4] and arg_2[5] == 'or':
                pass
            comparisons = arg_1[0]+' and ('+arg_2[0]+')', node.args[0]+' '+arg_1[1]+', '+arg_2[1],arg_1[2]+arg_2[2],arg_1[3]+arg_2[3],arg_1[4] or arg_2[4]
        if len(node.args) == 5:
            arg_1 = compile_py(node.args[2], ctx, property_='after')
            arg_2 = compile_py(node.args[3], ctx)
            arg_3 = compile_py(node.args[4], ctx, property_='until')
            comparisons = arg_1[0]+' and '+arg_2[0]+' and '+arg_3[0], node.args[0]+' '+arg_1[1]+', '+arg_2[1]+', '+arg_3[1], arg_1[2]+arg_2[2]+arg_3[2],arg_1[3]+arg_2[3]+arg_3[3],arg_1[4] or arg_2[4] or arg_3[4]
        if from_command:
            ctx.add_property(comparisons, node.args[1], node.args[0])
        return comparisons
    elif node.type == 'conjunction':
        if len(node.args) == 1:
            return compile_py(node.args[0], ctx, property_=property_)
        else:
            op,op_state,*_ = compile_py(node.args[0], ctx)
            expr_l,state_l,err1_l,err2_l,event_bool_l,event_l = compile_py(node.args[1], ctx, property_=property_)
            expr_r,state_r,err1_r,err2_r,event_bool_r,event_r = compile_py(node.args[2], ctx, property_=property_)
            prec = ''
            if op == 'implies':
                prec = 'not '
                op = 'or'
            return prec + str(expr_l) +' '+op+' ' + str(expr_r), str(state_l)+' '+op_state+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
    elif node.type == 'comparison':
        if len(node.args) == 1:
            return compile_py(node.args[0], ctx)
        op,op_state,*_ = compile_py(node.args[0], ctx)
        expr_l,state_l,err1_l,err2_l,event_bool_l,event_l = compile_py(node.args[1], ctx)
        expr_r,state_r,err1_r,err2_r,event_bool_r,event_r = compile_py(node.args[2], ctx)
        if len(node.args) == 4:
            if op == '==' or op == '!=':
                if op == '==':
                    op = '<='
                if op == '!=':
                    op = '>='
                return prop_prefix(property_) + str(expr_r)+'-'+str(node.args[3]) + op + str(expr_l) + op + str(expr_r)+'+'+str(node.args[3]),str(state_l)+' '+op_state+'{{'+str(node.args[3])+'}}'+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
            signal = '+'
            if property_ == 'always':
                if op == '>' or op == '>=':
                    signal = '-'
                return prop_prefix(property_) + str(expr_l) + op + str(expr_r) + signal + str(node.args[3]),str(state_l)+' '+op_state+'{{'+str(node.args[3])+'}}'+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
            if property_ == 'never':
                if op == '>' or op == '>=':
                    signal = '-'
                return prop_prefix(property_) + str(expr_l)+ signal + str(node.args[3]) + op + str(expr_r),str(state_l)+' '+op_state+'{{'+str(node.args[3])+'}}'+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
        return prop_prefix(property_) + str(expr_l) + op + str(expr_r), str(state_l)+' '+op_state+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
    elif node.type == 'multiplication':
        if len(node.args) == 1:
            return compile_py(node.args[0], ctx)
        op,*_ = compile_py(node.args[0], ctx)
        expr_l,state_l,err1_l,err2_l,event_bool_l,event_l = compile_py(node.args[1], ctx)
        expr_r,state_r,err1_r,err2_r,event_bool_r,event_r = compile_py(node.args[2], ctx)
        return str(expr_l) + op + str(expr_r), str(state_l)+' '+op+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
    elif node.type == 'addition':
        if len(node.args) == 1:
            return compile_py(node.args[0], ctx)
        op,*_ = compile_py(node.args[0], ctx)
        expr_l,state_l,err1_l,err2_l,event_bool_l,event_l = compile_py(node.args[1], ctx)
        expr_r,state_r,err1_r,err2_r,event_bool_r,event_r = compile_py(node.args[2], ctx)
        return str(expr_l) + op + str(expr_r), str(state_l)+' '+op+' '+str(state_r),err1_l+err1_r,err2_l+err2_r,event_bool_l or event_bool_r,[event_l[0]+event_r[0],event_l[1]+event_r[1],event_l[2]+event_r[2],event_l[3]+event_r[3]]
    elif node.type == 'operand':
        return compile_py(node.args[0], ctx)
    elif node.type == 'operand_paren':
        expr,state,err1,err2,event_bool,event = compile_py(node.args[0], ctx)
        return '('+expr+')','('+state+')',err1,err2,event_bool,event
    elif node.type == 'func':
        ctx.add_sim_subscriber()
        keep_state = node.args[0] + '.' + node.args[1]
        if len(node.args) > 2:
            args = compile_py(node.args[2], ctx)
            keep_state += '.' + '.'.join(args)
        else:
            args = []
        var_name = sim_funcs(node.args[0], node.args[1], args, ctx)
        return var_name, keep_state, [keep_state], [var_name],False,['','','','']
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + compile_py(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'temporalvalue':
        ctx.check_temporal_size(node.args[1])
        var_name, keep_state, *_ = compile_py(node.args[0], ctx)
        new_var_name = var_name.replace('[0]', f"[{node.args[1]}]")
        new_keep_state = '@{{' + keep_state + ', ' + str(node.args[1]) + '}}'
        return new_var_name, new_keep_state, [new_keep_state], [new_var_name],False,['','','','']