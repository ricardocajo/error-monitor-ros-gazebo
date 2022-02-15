""" A set of usefull classes and structures for the whole compiler """

from jinja2 import Environment, FileSystemLoader
from typing import Tuple, Dict

class Context(object):
    """ Save the context of a program """
    def __init__(self):
        self.stack = [{}]

class Emitter(object):
    """ Save the strings to be written in the monitor file """
    def __init__(self, _file_prefix):
        self.count = 0
        self.blocks = []
        self.file_loader = FileSystemLoader('templates')
        self.env = Environment(loader=self.file_loader)
        data = {'file_prefix': _file_prefix}
        self << ('file_beginning.python.jinja', data)

    def get_count(self):
        self.count += 1
        return self.count

    def get_id(self):
        id = self.get_count()
        return f"var{id}"

    def __lshift__(self, jinja_info: Tuple[str,Dict[str,str]]):
        template_file, data_dic = jinja_info
        template = self.env.get_template(template_file)
        output = template.render(data=data_dic)
        self.blocks.append(output)

    def get_code(self):
        template = self.env.get_template('join_code.python.jinja')
        return template.render(code=self.blocks)

class Node(object):
    """ The ast of a program """
    def __init__(self, t, *args):
        self.type = t
        self.args = args

    def __str__(self):
        s = "rule: " + str(self.type) + "\n"
        s += "".join(["i: " + str(i) + "\n" for i in self.args])
        return s


""" The language possible tokens """

func_main_token_type = 'FUNC_MAIN'
func_main = {
    'position_x' : 'POSITION_X',
    'position_y' : 'POSITION_Y',
    'position_z' : 'POSITION_Z',
    'velocity' : 'VELOCITY',
    'distance' : 'DISTANCE',
    'localization_error' : 'LOCALIZATION_ERROR',
    'orientation' : 'ORIENTATION'
}

reserved = {
    'true' : 'TRUE',
    'false' : 'FALSE',
    'decl' : 'DECL',
    'model' : 'MODEL',
    'always' : 'ALWAYS',
    'never' : 'NEVER',
    'eventually' : 'EVENTUALLY',
    'after' : 'AFTER',
    'until' : 'UNTIL',
    'implies' : 'IMPLIES',
    'not' : 'NOT',
    'and' : 'AND',
    'or' : 'OR'
}

literals = ['>','<','(',')','+','-','*','/','{','}','@','=',':',',','.']

tokens = ['NAME','TOPIC_NAME','INTEGER','FLOAT','EQ','DIF','GTE',
         'LEE',func_main_token_type] + list(reserved.values()) + list(func_main.values())