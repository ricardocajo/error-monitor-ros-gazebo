""" The language grammar """

import tokens as tok
from utils import *

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

precedence = (
    ('nonassoc','GTE','LEE','EQ','DIF','>','<'),
)

#value{x} missing

def p_program(p):
    '''program : command
               | command program'''
    if len(p) > 2:
        p[0] = Node('program', p[1], p[2])
    else:
        p[0] = Node('program', p[1])
    print("teste1")
    print(p[0])

def p_command(p):
    '''command : declaration
               | model
               | pattern
               | association'''
    p[0] = Node('command', p[1])
    print("teste2")
    print(p[0])

def p_declaration(p):
    '''declaration : DECL NAME TOPIC_NAME msgtype
                   | DECL NAME NAME msgtype'''
    p[0] = Node('declaration', p[1], p[2], p[3], p[4])
    print("teste3")
    print(p[0])

def p_model(p):
    '''model : MODEL NAME ':' NAME TOPIC_NAME msgtype
             | MODEL NAME ':' NAME NAME msgtype'''
    p[0] = Node('model', p[1], p[2], p[3], p[4], p[5], p[6])
    print("teste4")
    print(p[0])

def p_msgtype(p):
    '''msgtype : NAME
               | NAME '.' msgtype'''
    if len(p) > 2:
        p[0] = Node('msgtype', p[1], p[2], p[3])
    else:
        p[0] = Node('msgtype', p[1])
    print("teste5")
    print(p[0])

def p_association(p):
    '''association : NAME '=' expression'''
    p[0] = Node('association', p[1], p[2], p[3])
    print("teste6")
    print(p[0])

def p_pattern_4(p):
    '''pattern : ALWAYS '(' paargs ')'
               | NEVER '(' paargs ')'
               | EVENTUALLY '(' paargs ')'
               | NOT '(' paargs ')' '''
    p[0] = Node('pattern_4', p[1], p[2], p[3], p[4])
    print("teste7")
    print(p[0])

def p_pattern_6(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' '''
    p[0] = Node('pattern_6', p[1], p[2], p[3], p[4], p[5], p[6])
    print("teste8")
    print(p[0])

def p_pattern_7(p):
    '''pattern : '(' paargs ')' UNTIL '(' paargs ')'
               | '(' paargs ')' IMPLIES '(' paargs ')'
               | '(' paargs ')' AND '(' paargs ')'
               | '(' paargs ')' OR '(' paargs ')' '''
    p[0] = Node('pattern_7', p[1], p[2], p[3], p[4], p[5], p[6], p[7])
    print("teste9")
    print(p[0])

def p_pattern_10(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' UNTIL '(' paargs ')' '''
    p[0] = Node('pattern_10', p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10])
    print("teste10")
    print(p[0])

def p_paargs(p):
    '''paargs : pattern
              | comparison
              | NAME'''
    p[0] = Node('paargs', p[1])
    print("teste11")
    print(p[0])

def p_comparison(p):
    '''comparison : operand '>' operand
                  | operand '<' operand
                  | operand GTE operand
                  | operand LEE operand
                  | operand EQ operand
                  | operand DIF operand'''
    p[0] = Node('comparison', p[1], p[2], p[3])
    print("teste12")
    print(p[0])

def p_expression(p):
    '''expression : operand '+' operand
                  | operand '-' operand
                  | operand '*' operand
                  | operand '/' operand'''
    p[0] = Node('expression', p[1], p[2], p[3])
    print("teste12.5")
    print(p[0])

def p_operand(p):
    '''operand : func
               | INTEGER
               | FLOAT
               | NAME
               | temporalvalue
               | comparison'''
    p[0] = Node('operand', p[1])
    print("teste14")
    print(p[0])

def p_func_onearg(p):
    '''func : POSITION_X NAME
            | POSITION_Y NAME
            | POSITION_Z NAME
            | ORIENTATION NAME
            | VELOCITY NAME
            | LOCALIZATION_ERROR NAME'''
    p[0] = Node('func_onearg', p[1], p[2])
    print("teste15")
    print(p[0])

def p_func_twoarg(p):
    '''func : DISTANCE NAME NAME'''
    p[0] = Node('func_twoarg', p[1], p[2], p[3])
    print("teste16")
    print(p[0])

def p_temporalvalue(p):
    '''temporalvalue : '@' '{' NAME ',' INTEGER '}' '''
    p[0] = Node('func_twoarg', p[1], p[2], p[3], p[4], p[5], p[6])
    print("teste17")
    print(p[0])

def p_error(p):
    print("Syntax error at '%s'. Line number '%d'" % (p.value, p.lineno))