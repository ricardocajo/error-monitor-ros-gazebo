""" The language grammar """

import tokens as tok
from utils import *

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

precedence = (
    ('left','UNTIL','IMPLIES','AND','OR','ALWAYS','EVENTUALLY','NOT'),
    ('nonassoc','GTE','LEE','EQ','DIF','>','<'),
)

#comparison needs to be non-associative?
def p_opargs(p):
    '''opargs : operator
              | comparison'''
    p[0] = Node('opargs', p[1])
    #print("teste1")
    #print(p[0])

def p_operator_5(p):
    '''operator : ALWAYS '(' opargs ')'
                | EVENTUALLY '(' opargs ')'
                | NOT '(' opargs ')' '''
    p[0] = Node('operator_5', p[1], p[2], p[3], p[4])
    #print("teste3")
    #print(p[0])

def p_operator_8(p):
    '''operator : '(' opargs ')' UNTIL '(' opargs ')'
                | '(' opargs ')' IMPLIES '(' opargs ')'
                | '(' opargs ')' AND '(' opargs ')'
                | '(' opargs ')' OR '(' opargs ')' '''
    p[0] = Node('operator_8', p[1], p[2], p[3], p[4], p[5], p[6], p[7])
    #print("teste2")
    #print(p[0])

def p_comparison(p):
    '''comparison : func opbin NUMBER
                  | NUMBER opbin func
                  | func opbin func'''
    p[0] = Node('comparison', p[1], p[2], p[3])
    #print("teste4")
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