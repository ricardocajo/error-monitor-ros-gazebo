import language_lex
import ply.lex as lex
import language_yacc
import ply.yacc as yacc
from utils import Context
from verify import *
from interpret_ros_py import *

lexer = lex.lex(module=language_lex)
parser = yacc.yacc(module=language_yacc, outputdir="out_files")

"""              TEST LEXER
data = '''always (((position_x robot1) > 0.0) and ((position_y robot1) > 0.0))'''
lexer.input(data)
while True:
    tok = lexer.token()
    if not tok:
        break
    print(tok)
"""
import sys
filename = sys.argv[1]
f = open(filename, 'r')
_input = f.read()
ast = parser.parse(lexer=lexer, input=_input)
print(ast)
#"""
verify(Context(),ast)
interpret_ros_py(Context(),ast)