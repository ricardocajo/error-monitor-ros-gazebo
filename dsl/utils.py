""" A set of usefull classes and structures for the whole compiler """

from jinja2 import Environment, FileSystemLoader
from typing import Tuple, Dict

class Context(object):
    """ Save the context of a program """
    def __init__(self):
        self.stack = [{}]
        self.subscribers = [] #'topic','msgtype','library'
        self.sim_subscriber = False
        self.properties = [] #

    def get_subscribers(self):
        return self.subscribers

    def add_subscriber(self, topic, msgtype, library):
        sub_data = {'topic': topic, 'msgtype': msgtype, 'library': library}
        self.subscribers.append(sub_data)

    def add_sim_subscriber(self):
        if self.sim_subscriber is False:
            self.subscribers = [{'topic': '/gazebo/model_states', 'msgtype': 'ModelStates', 'library': 'gazebo_msgs'}] + self.subscribers
        self.sim_subscriber = True

    def get_property(self):
        return self.properties

    def add_property(self):
        #TODO
        pass
    
    def __str__(self):
        return str(self.subscribers)

class Emitter(object):
    """ Save the strings to be written in the monitor file """
    def __init__(self, _file_prefix):
        self.count = 0
        self.blocks = []
        self.file_loader = FileSystemLoader('templates')
        self.env = Environment(loader=self.file_loader,extensions=['jinja2.ext.do'])
        data = {'file_prefix': _file_prefix}
        self << ('file_beginning.jinja', data)

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
        template = self.env.get_template('join_code.jinja')
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

# The default functions of the language
func_main = {
    'position' : '1',
    'velocity' : '1',
    'distance' : '2',
    'localization_error' : '1',
    'orientation' : '1'
}
func_main_list = list(func_main.keys())

""" The tokens of the language """
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
         'LEE','FUNC_MAIN'] + list(reserved.values())