from __future__ import annotations

import os
import stat
import sys
from pathlib import Path

import ply.lex as lex
import ply.yacc as yacc

import language_lex
import language_yacc
from compile_py import *
from type_checker import *
from utils import *

# run - python language.py test.txt /home/rcordeiro/ros_workspace/src/test_pkg/src

lexer = lex.lex(module=language_lex)
parser = yacc.yacc(module=language_yacc, outputdir="parse_files")

#             TEST LEXER
# data = ''' model turtlebot3_burger:
#    positiona /odom Odometry.pose.pose.position
#   ; '''
# lexer.input(data)
# while True:
#    tok = lexer.token()
#    if not tok:
#        break
#    print(tok)

filename = sys.argv[1]
file_prefix = filename[:-4]
ros_package_dir_path = sys.argv[2]
if not Path(ros_package_dir_path).is_dir():
    print("The given directory for a ROS package doesn't exist.")
    sys.exit()
f = open(filename)
_input = f.read()
filename = file_prefix + ".py"
filepath = os.path.join(ros_package_dir_path, filename)
try:
    ast = parser.parse(lexer=lexer, input=_input)
    type_checker(ast, filepath=ros_package_dir_path)
    code = compile_py(ast, file_prefix=file_prefix, filepath=ros_package_dir_path)
    with open(filepath, "w") as f_out:
        f_out.write(code)
        os.chmod(filepath, stat.S_IRWXU)
except TypeError as e:
    print(e)
# """
