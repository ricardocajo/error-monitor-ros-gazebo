import language_lex
import ply.lex as lex
import language_yacc
import ply.yacc as yacc
from utils import *
from type_checker import *
from compile_py import *
from pathlib import Path
import os, stat, sys
#run - python language.py test.txt /home/rcordeiro/ros_workspace/src/test_pkg/src

lexer = lex.lex(module=language_lex)
parser = yacc.yacc(module=language_yacc, outputdir="parse_files")

"""              TEST LEXER
data = '''model turtlebot3_burger:
    laser_position /odom Odometry.pose.pose
    own_velocity /cmd_vel Twist.linear.x
    ;'''
lexer.input(data)
while True:
    tok = lexer.token()
    if not tok:
        break
    print(tok)
"""

filename = sys.argv[1]
file_prefix = filename[:-4]
ros_package_dir_path = sys.argv[2]
if not Path(ros_package_dir_path).is_dir():
    print("The given directory for a ROS package doesn't exist.")
    sys.exit()
f = open(filename, 'r')
_input = f.read()
filename = file_prefix + ".py"
filepath = os.path.join(ros_package_dir_path, filename)
ast = parser.parse(lexer=lexer, input=_input)
type_checker(TypeCheckerContext(), ast, ros_package_dir_path)
code = compile_py(ast, file_prefix=file_prefix, filepath=ros_package_dir_path)
with open(filepath, "w") as f_out:
    f_out.write(code)
    os.chmod(filepath, stat.S_IRWXU)
#"""