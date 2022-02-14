""" The language grammar """

import tokens as tok
from utils import *

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

precedence = (
    ('nonassoc','GTE','LEE','EQ','DIF','>','<'),
    ('left','+','-'),
    ('left','/','*'),
)

#value{x} missing

def p_program(p):
    '''program : command
               | command program'''
    if len(p) > 2:
        p[0] = Node('program', p[1], p[2])
    else:
        p[0] = Node('program', p[1])

def p_command(p):
    '''command : declaration
               | model
               | pattern
               | association'''
    p[0] = Node('command', p[1])

def p_declaration(p):
    '''declaration : DECL NAME TOPIC_NAME msgtype
                   | DECL NAME NAME msgtype'''
    p[0] = Node('declaration', p[1], p[2], p[3], p[4])

def p_model(p):
    '''model : MODEL NAME ':' NAME TOPIC_NAME msgtype
             | MODEL NAME ':' NAME NAME msgtype'''
    p[0] = Node('model', p[1], p[2], p[3], p[4], p[5], p[6])

def p_msgtype(p):
    '''msgtype : NAME
               | NAME '.' msgtype'''
    if len(p) > 2:
        p[0] = Node('msgtype', p[1], p[2], p[3])
    else:
        p[0] = Node('msgtype', p[1])

def p_association(p):
    '''association : NAME '=' expression'''
    p[0] = Node('association', p[1], p[2], p[3])

def p_expression(p):
    '''expression : operation
                  | comparison
                  | operand
                  | operation '{' number '}'
                  | operand '{' number '}' '''
    if len(p) > 2:
        p[0] = Node('expression', p[1], p[2], p[3], p[4])
    else:
        p[0] = Node('expression', p[1])

def p_pattern_4(p):
    '''pattern : ALWAYS '(' paargs ')'
               | NEVER '(' paargs ')'
               | EVENTUALLY '(' paargs ')'
               | NOT '(' paargs ')' '''
    p[0] = Node('pattern_4', p[1], p[2], p[3], p[4])

def p_pattern_6(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' '''
    p[0] = Node('pattern_6', p[1], p[2], p[3], p[4], p[5], p[6])

def p_pattern_7(p):
    '''pattern : '(' paargs ')' UNTIL '(' paargs ')'
               | '(' paargs ')' IMPLIES '(' paargs ')' '''
    p[0] = Node('pattern_7', p[1], p[2], p[3], p[4], p[5], p[6], p[7])
    print("teste9")
    print(p[0])

def p_patter_multi(p):
    '''pattern : '(' paargs ')' AND '(' paargs ')' pattsup
               | '(' paargs ')' OR '(' paargs ')' pattsup'''
    p[0] = Node('pattern_multi', p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8])

def p_pattsup(p):
    '''pattsup : AND '(' paargs ')' 
               | OR '(' paargs ')' 
               | empty'''
    if len(p) > 2:
        p[0] = Node('pattsup', p[1], p[2], p[3], p[4])
    else:  #TODO maybe this will give an error whiel check None type?
        pass

def p_pattern_10(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' UNTIL '(' paargs ')' '''
    p[0] = Node('pattern_10', p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10])

def p_paargs(p):
    '''paargs : pattern
              | comparison
              | NAME'''
    p[0] = Node('paargs', p[1])

def p_operation(p):
    '''operation : expression '+' expression
                 | expression '-' expression
                 | expression '/' expression
                 | expression '*' expression'''
    p[0] = Node('operation', p[1], p[2], p[3])           

def p_comparison(p):
    '''comparison : expression '>' expression
                  | expression '<' expression
                  | expression GTE expression
                  | expression LEE expression
                  | expression EQ expression
                  | expression DIF expression'''
    p[0] = Node('comparison', p[1], p[2], p[3])
    print("teste13")
    print(p[0])

def p_number(p):
    '''number : FLOAT
              | INTEGER'''
    p[0] = Node('number', p[1])

def p_operand(p):
    '''operand : func
               | number
               | NAME
               | bool
               | temporalvalue'''
    p[0] = Node('operand', p[1])

def p_bool(p):
    '''bool : TRUE
            | FALSE'''
    p[0] = Node('bool', p[1])

def p_func_onearg(p):
    '''func : POSITION_X NAME
            | POSITION_Y NAME
            | POSITION_Z NAME
            | ORIENTATION NAME
            | VELOCITY NAME
            | LOCALIZATION_ERROR NAME'''
    p[0] = Node('func_onearg', p[1], p[2])

def p_func_twoarg(p):
    '''func : DISTANCE NAME NAME'''
    p[0] = Node('func_twoarg', p[1], p[2], p[3])

def p_temporalvalue(p):
    '''temporalvalue : '@' '{' NAME ',' INTEGER '}' '''
    p[0] = Node('temporalvalue', p[1], p[2], p[3], p[4], p[5], p[6])

def p_empty(p):
    'empty :'
    pass

def p_error(p):
    print("Syntax error at '%s'. Line number '%d'" % (p.value, p.lineno))