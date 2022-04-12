from utils import *
import os, subprocess

def type_checker(node: Node, ctx=None, filepath=None, property_=None, line=None, from_command=False, object_=None):
    """ Verifies the syntax of the language"""
    if ctx is None:
        ctx = TypeCheckerContext()
        type_checker(node, ctx)
    elif not type(node) is Node:
        return '', str(node), False
    elif node.type == 'program':
        if len(node.args) > 1:
            type_checker(node.args[1], ctx)
        type_checker(node.args[0], ctx)
    elif node.type == 'command':
        type_checker(node.args[0], ctx, from_command=True)
    elif node.type == 'declaration':
        ctx.add_topic_value(node.args[0])
    elif node.type == 'model':
        #print('model')
        type_checker(node.args[1], ctx, object_=node.args[0])
    elif node.type == 'modelargs':
        if node.args[0] == 'localization_error' or node.args[0] == 'distance':
            raise TypeError(f"'{node.args[0]}' is not a valid object attribute")
        if node.args[0] in funcs:
            ctx.add_model(object_, node.args[0])
        else:
            ctx.add_topic_value(node.args[0])
        if len(node.args) > 3:
            type_checker(node.args[3], ctx, object_=object_)
    elif node.type == 'association':
        type_, state, is_event = type_checker(node.args[1], ctx)
        ctx.add_assoc({'var': node.args[0], 'type': type_})
    elif node.type == 'property':
        #print('property')
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        if from_command:
            property_ = node.args[0]
            line = node.args[1]
        type_1, state_1, is_event_1 = type_checker(node.args[2], ctx, property_=node.args[0], line=node.args[1])
        if not type_1 == 'conjunction' and not type_1 == 'comparison' and not type_1 == 'property':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_1}' should be fiting for a property")
        if len(node.args) == 3:
            eventually_check = False
            if node.args[0] == 'eventually':
                eventually_check = True
            return 'property', node.args[0]+' '+state_1, eventually_check
        if len(node.args) == 4:
            type_2, state_2, is_event_2 = type_checker(node.args[3], ctx, property_=node.args[0], line=node.args[1])
            if not type_2 == 'conjunction' and not type_2 == 'comparison' and not type_2 == 'property':
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_2}' should be fiting for a property")
            if is_event_1:
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_1}' (first argument) can't have an 'eventually' property")
            return 'property', node.args[0]+' '+state_1+', '+state_2, is_event_1 or is_event_2
        if len(node.args) == 5:
            type_2, state_2, is_event_2 = type_checker(node.args[3], ctx, property_=node.args[0], line=node.args[1])
            if not type_2 == 'conjunction' and not type_2 == 'comparison' and not type_2 == 'property':
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_2}' should be fiting for a property")
            type_3, state_3, is_event_3 = type_checker(node.args[4], ctx, property_=node.args[0], line=node.args[1])
            if not type_3 == 'conjunction' and not type_3 == 'comparison' and not type_3 == 'property':
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_3}' should be fiting for a property")
            if is_event_1:
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_1}' (first argument) can't have an 'eventually' property")
            if is_event_2:
                raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_2}' (second argument) can't have an 'eventually' property")
            return 'property', node.args[0]+' '+state_1+', '+state_2+', '+state_3, is_event_1 or is_event_2 or is_event_3
    elif node.type == 'conjunction':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op,_ = type_checker(node.args[0], ctx)
        type_l, state_l, is_event_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        #if not type_l == 'property' and not type_l == 'comparison' and not type_l == 'conjunction':
        #    raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for conjunction")
        type_r, state_r, is_event_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        #if not type_l == 'property' and not type_r == 'comparison':
        #    raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for conjunction")
        return 'conjunction', state_l+' '+op+' '+state_r, is_event_l or is_event_r
    elif node.type == 'comparison':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op,_ = type_checker(node.args[0], ctx)
        type_l, state_l, is_event_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'multiplication' and not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for comparison")
        type_r, state_r, is_event_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'multiplication' and not type_r == 'addition' and not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for comparison")
        return 'comparison', state_l+' '+op+' '+state_r, is_event_l or is_event_r
    elif node.type == 'multiplication':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op,_ = type_checker(node.args[0], ctx)
        type_l, state_l, is_event_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'multiplication' and not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for multiplication")
        type_r, state_r, is_event_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'addition' and not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for multiplication")
        return 'multiplication', state_l+' '+op+' '+state_r, is_event_l or is_event_r
    elif node.type == 'addition':
        if len(node.args) == 1:
            return type_checker(node.args[0], ctx, property_=property_, line=line)
        _,op,_ = type_checker(node.args[0], ctx)
        type_l, state_l, is_event_l = type_checker(node.args[1], ctx, property_=property_, line=line)
        if not type_l == 'addition' and not type_l == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_l}' should be fiting for addition")
        type_r, state_r, is_event_r = type_checker(node.args[2], ctx, property_=property_, line=line)
        if not type_r == 'operand':
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    '{state_r}' should be fiting for addition")
        return 'addition', state_l+' '+op+' '+state_r, is_event_l or is_event_r
    elif node.type == 'operand':
        _, state, is_event = type_checker(node.args[0], ctx, property_=property_, line=line)
        return 'operand', state, is_event
    elif node.type == 'operand_name':
        print('i entered operand_name')
        _, state, is_event = type_checker(node.args[0], ctx, property_=property_, line=line)
        is_var, type_ = ctx.is_assoc(node.args[0])
        if is_var:
            return type_, state, is_event
        if not ctx.is_topic_value(node.args[0]):
            raise TypeError(f"Syntactic error in '{property_}' property at line {str(line)}:\n    'variable {node.args[0]} referenced before assignment")
        return 'operand', state, is_event
    elif node.type == 'operand_paren':
        return type_checker(node.args[0], ctx, property_=property_, line=line)
    elif node.type == 'func':
        if node.args[1] == 'localization_error':
            if not ctx.is_model(node.args[0], 'position'):
                raise TypeError(f"'localization_error' function used but object '{node.args[0]}' has no model for 'position'")
        state = node.args[0] + '.' + node.args[1]
        if len(node.args) > 2:
            args = type_checker(node.args[2], ctx)
            state += '.' + '.'.join(args)
        return 'operand', state, False
    elif node.type == 'funcargs':
        if len(node.args) > 1:
            return [node.args[0]] + type_checker(node.args[1], ctx)
        return [node.args[0]]
    elif node.type == 'temporalvalue':
        _,state = type_checker(node.args[0], ctx)
        new_state = '@{{' + state + ', ' + str(node.args[1]) + '}}'
        return 'operand', new_state, False