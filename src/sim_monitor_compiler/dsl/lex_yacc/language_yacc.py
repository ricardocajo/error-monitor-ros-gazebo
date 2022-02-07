import tokens as tok

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

def p_test(p):
    '''operator : ALWAYS '(' '(' '(' POSITION_X NAME ')' '>' FLOAT ')' AND '(' '(' POSITION_Y NAME ')' '>' FLOAT ')' ')'
    '''
    p[0] = p[1] + p[2]

def p_error(p):
    print("Syntax error")