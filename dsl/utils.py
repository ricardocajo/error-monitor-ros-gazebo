""" A set of usefull classes and structures for the whole compiler """

from jinja2 import Environment, FileSystemLoader
from typing import Tuple, Dict

class Context(object):
    """ Save the context of a program """
    def __init__(self):
        self.stack = [{}]
        self.subscribers = [] # 'topic','msgtype','library','sub_name'
        self.sim_subscriber = False
        self.vars = [] # 'name','object_name','args','sim'
        self.properties = [] #

    def get_subscribers(self):
        return self.subscribers

    def add_subscriber(self, topic, msgtype, library, sub_name):
        sub_data = {'topic': topic, 'msgtype': msgtype, 'library': library, 'sub_name': sub_name}
        self.subscribers.append(sub_data)

    def add_sim_subscriber(self):
        if self.sim_subscriber is False:
            self.add_subscriber('/gazebo/model_states', 'ModelStates', 'gazebo_msgs', 'model_states_sub')
            self.sim_subscriber = True

    def get_sim_subscriber(self):
        return self.sim_subscriber

    def get_vars(self):
        return self.vars

    def add_var(self, name, object_name, sim, arg, arg_extra=None):
        var_data = {'name': name, 'object_name': object_name, 'sim': sim, 'arg': arg, 
                    'arg_extra': arg_extra}
        self.vars.append(var_data)

    def get_property(self):
        return self.properties

    def add_property(self):
        #TODO
        pass
    
    def __str__(self):
        return str(self.subscribers)

def compile_ros_py(ctx: Context, file_prefix: str):
    """ Creates a python script capable of running the associated monitor code in ROS """
    file_loader = FileSystemLoader('templates')
    env = Environment(loader=file_loader,extensions=['jinja2.ext.do'])
    template = env.get_template('program.jinja')
    return template.render(file_prefix=file_prefix, sim_sub=ctx.get_sim_subscriber(), 
                           subscribers=ctx.get_subscribers(), var_list=ctx.get_vars())

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