import language_lex
import ply.lex as lex
import language_yacc
import ply.yacc as yacc
from utils import *
from verify_language import *
from compile_language import *
from pathlib import Path

lexer = lex.lex(module=language_lex)
parser = yacc.yacc(module=language_yacc, outputdir="parse_files")

"""              TEST LEXER
data = '''always (((position_x robot1) >= 0.0) and ((position_y robot1) > 0.0))'''
lexer.input(data)
while True:
    tok = lexer.token()
    if not tok:
        break
    print(tok)
"""
#run - python language.py test.txt /home/rcordeiro/ros_workspace/src/test_pkg/src
import sys
filename = sys.argv[1]
file_prefix = filename[:-4]
#for test: /home/rcordeiro/ros_workspace/src/test_pkg   /src and /launch
ros_package_dir_path = sys.argv[2]

if not Path(ros_package_dir_path).is_dir():
    print("The given directory for a ROS package doesn't exist.")
    sys.exit()

f = open(filename, 'r')
_input = f.read()
ast = parser.parse(lexer=lexer, input=_input)
#print(ast)
verify(Context(),ast)
code = compile_ros_py(Context(), ast, file_prefix)
#filename = file_prefix + ".py"
#filepath = os.path.join(ros_package_dir_path, filename)
#with open(filepath, "w") as f_out:
#    f_out.write(code)
#"""
