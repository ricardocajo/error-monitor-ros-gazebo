from utils import *
import os, subprocess

def type_checker(node: Node, ctx=None, filepath=None, property_=None, line=None):
    """ Verifies the syntax of the language"""
    if ctx is None:
        ctx = TypeCheckerContext()
        type_checker(node, ctx)
    elif not type(node) is Node:
        return '',str(node)
    elif node.type == 'program':
        if len(node.args) > 1:
            type_checker(node.args[1], ctx)
        type_checker(node.args[0], ctx)
    elif node.type == 'command':
        type_checker(node.args[0], ctx)
    elif node.type == 'declaration':
        ctx.add_topic_value(node.args[0])
    elif node.type == 'model':
        type_checker(node.args[1], ctx)
    elif node.type == 'modelargs':
        if len(node.args) > 3:
            type_checker(node.args[3], ctx)
        ctx.add_topic_value(node.args[0])
    elif node.type == 'association':
        ctx.add_assoc(node.args[0])
    elif node.type == 'property':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        if len(node.args) == 3:
            type_1, state_1 = type_checker(node.args[2], ctx, property_=node.args[0], line=node.args[1])
            return 'property', node.args[0]+' '+state_1
        if len(node.args) == 4:
            type_1, state_1 = type_checker(node.args[2], ctx, property_=node.args[0], line=node.args[1])
            type_2, state_2 = type_checker(node.args[3], ctx, property_=node.args[0], line=node.args[1])
            return 'property', node.args[0]+' '+state_1+', '+state_2
        if len(node.args) == 5:
            type_1, state_1 = type_checker(node.args[2], ctx, property_=node.args[0], line=node.args[1])
            type_2, state_2 = type_checker(node.args[3], ctx, property_=node.args[0], line=node.args[1])
            type_3, state_3 = type_checker(node.args[4], ctx, property_=node.args[0], line=node.args[1])
            return 'property', node.args[0]+' '+state_1+', '+state_2+', '+state_3
#    elif node.type == 'conjunction':  TODO check this after correcting grammar
#        if len(node.args) == 1:
#            return type_checker(node.args[0], ctx, property_=property_, line=line)
#        _,op = type_checker(node.args[0], ctx)
#        type_l, state_l = type_checker(node.args[1], ctx, property_=property_, line=line)
#        if not type_l == 'property' and not type_l == 'comparison' and not type_l == 'conjunction':
#            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for conjunction.")
#        type_r, state_r = type_checker(node.args[2], ctx, property_=property_, line=line)
#        if not type_l == 'property' and not type_r == 'comparison':
#            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for conjunction.")
#        return 'conjunction', state_l+' '+op+' '+state_r
    elif node.type == 'comparison':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op = type_checker(node.args[0], ctx)
        type_l, state_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'multiplication' and not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for comparison.")
        type_r, state_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'multiplication' and not type_r == 'addition' and not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for comparison.")
        return 'comparison', state_l+' '+op+' '+state_r
    elif node.type == 'multiplication':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op = type_checker(node.args[0], ctx)
        type_l, state_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'multiplication' and not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for multiplication.")
        type_r, state_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'addition' and not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for multiplication.")
        return 'multiplication', state_l+' '+op+' '+state_r
    elif node.type == 'addition':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op = type_checker(node.args[0], ctx)
        type_l, state_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for addition.")
        type_r, state_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for addition.")
        return 'addition', state_l+' '+op+' '+state_r
    elif node.type == 'operand':
        _, state = type_checker(node.args[0], ctx, property_=property_, line=line)
        return 'operand', state
    elif node.type == 'operand_paren':
        return type_checker(node.args[0], ctx, property_=property_, line=line)
    elif node.type == 'func':
        state = node.args[0] + '.' + node.args[1]
        if len(node.args) > 2:
            args = type_checker(node.args[2], ctx)
            state += '.' + '.'.join(args)
        return 'operand', state
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + type_checker(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'temporalvalue':
        _,state = type_checker(node.args[0], ctx)
        new_state = '@{{' + state + ', ' + str(node.args[1]) + '}}'
        return 'operand', new_state