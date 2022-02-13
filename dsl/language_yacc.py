""" The language grammar """

import tokens as tok
from utils import *

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

precedence = (
    ('nonassoc','GTE','LEE','EQ','DIF','>','<'),
)

def p_declaration(p):
    '''declaration : DECL NAME TOPIC_NAME msgtype
                   | DECL NAME NAME msgtype'''
    p[0] = Node('declaration', p[1], p[2], p[3], p[4])
    #print("teste2")
    #print(p[0])

def p_msgtype(p):
    '''msgtype : NAME
               | NAME '.' msgtype'''
    p[0] = Node('declaration', p[1], p[2], p[3], p[4])
    #print("teste2")
    #print(p[0])
    
def p_pattern_4(p):
    '''pattern : ALWAYS '(' opargs ')'
               | NEVER '(' opargs ')'
               | EVENTUALLY '(' opargs ')'
               | NOT '(' opargs ')' '''
    p[0] = Node('pattern_4', p[1], p[2], p[3], p[4])
    #print("teste3")
    #print(p[0])

def p_pattern_6(p):
    '''pattern : AFTER '(' opargs ',' opargs ')' '''
    p[0] = Node('pattern_6', p[1], p[2], p[3], p[4], p[5], p[6])
    #print("teste2")
    #print(p[0])

def p_pattern_7(p):
    '''pattern : '(' opargs ')' UNTIL '(' opargs ')'
               | '(' opargs ')' IMPLIES '(' opargs ')'
               | '(' opargs ')' AND '(' opargs ')'
               | '(' opargs ')' OR '(' opargs ')' '''
    p[0] = Node('pattern_7', p[1], p[2], p[3], p[4], p[5], p[6], p[7])
    #print("teste2")
    #print(p[0])

def p_pattern_10(p):
    '''pattern : AFTER '(' opargs ',' opargs ')' UNTIL '(' opargs ')' '''
    p[0] = Node('pattern_10', p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10])
    #print("teste2")
    #print(p[0])

def p_opbin(p):
    '''opbin : '+'
             | '-'
             | '>'
             | '<'
             | '*'
             | '/'
             | GTE
             | LEE
             | EQ
             | DIF'''
    p[0] = Node('opbin', p[1])
    #print("teste5")
    #print(p[0])

def p_func_onearg(p):
    '''func : POSITION_X NAME
            | POSITION_Y NAME
            | POSITION_Z NAME
            | ORIENTATION NAME
            | VELOCITY NAME
            | LOCALIZATION_ERROR NAME'''
    p[0] = Node('func_onearg', p[1], p[2])
    #print("teste6")
    #print(p[0])

def p_func_twoarg(p):
    '''func : DISTANCE NAME NAME'''
    p[0] = Node('func_twoarg', p[1], p[2], p[3])
    #print("teste7")
    #print(p[0])

def p_error(p):
    print("Syntax error")