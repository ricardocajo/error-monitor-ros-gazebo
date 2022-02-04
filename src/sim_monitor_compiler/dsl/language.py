import language_lex
import ply.lex as lex
import ply.yacc as yac

lexer = lex.lex(module=language_lex)

#"""              TEST LEXER
data = '''always (((position_x robot1) > 0.0) and ((position_y robot1) > 0.0))'''
lexer.input(data)
while True:
    tok = lexer.token()
    if not tok:
        break
    print(tok)
#"""

def p_test(t):
    '''operator : ALWAYS ( ( ( POSITION_X NAME ) > FLOAT ) AND ( ( POSITION_Y NAME ) > FLOAT ) )'''

parser = yacc.yacc()