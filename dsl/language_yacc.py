""" The language grammar """

import tokens as tok

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

#comparison needs to be non-associative?
def p_opargs(p):
    '''opargs : operator
              | comparison'''

def p_operator(p):
    '''operator : ALWAYS opargs
                | EVENTUALLY opargs
                | opargs UNTIL opargs
                | opargs IMPLIES opargs
                | opargs AND opargs
                | opargs OR opargs
                | NOT opargs'''

def p_comparison(p):
    '''comparison : func opbin FLOAT
                  | FLOAT opbin func'''

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

def p_func_onearg(p):
    '''func : POSITION_X NAME
            | POSITION_Y NAME
            | ORIENTATION NAME
            | VELOCITY NAME
            | LOCALIZATION_ERROR NAME'''

def p_func_twoarg(p):
    '''func : DISTANCE NAME NAME'''

def p_error(p):
    print("Syntax error")