import tokens as tok

tokens = tok.tokens
literals = tok.literals
reserved = tok.reserved

def p_test(p):
    '''operator : ALWAYS '(' '(' '(' POSITION_X NAME ')' '>' FLOAT ')' AND '(' '(' POSITION_Y NAME ')' '>' FLOAT ')' ')'
    '''
    p[0] = p[1] + p[2] + p[3] + p[4] + p[5] + p[6] + p[7] + p[8] + str(p[9]) + p[10] + p[11] + p[12] + p[13] + p[14] + p[15] + p[16] + p[17] + str(p[18]) + p[19] + p[20]

def p_error(p):
    print("Syntax error")