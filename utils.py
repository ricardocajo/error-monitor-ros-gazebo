""" A set of usefull classes and structures for the whole compiler """

from jinja2 import FileSystemLoader, Environment
import subprocess

class TypeCheckerContext(object):
    """ Save the context of a program (in the paradigm of the type_checker function)"""
    def __init__(self):
        self.stack = [{}]

    def __str__(self):
        return "implement"

class CompileContext(object):
    """ Save the context of a program (in the paradigm of the compile_py function)"""
    def __init__(self, file_prefix, filepath):
        self.file_prefix = file_prefix
        self.filepath = filepath
        self.subscribers = []
        self.sim_subscriber = False
        self.vars = []
        self.assoc = []
        self.properties = []
        self.property_counter = 1

    def add_subscriber(self, topic, msgtype, library, sub_name):
        sub_data = {'topic': topic, 'msgtype': msgtype, 'library': library, 'sub_name': sub_name}
        self.subscribers.append(sub_data)

    def add_sim_subscriber(self):
        if self.sim_subscriber is False:
            self.add_subscriber('/gazebo/model_states', 'ModelStates', 'gazebo_msgs', 'model_states')
            self.sim_subscriber = True

    def add_var(self, name, extract):
        if not added_var(name, self.vars):
            var_data = {'name': name, 'extract': extract}
            self.vars.append(var_data)

    def add_assoc(self, assoc_var_name, expr_var_name):
        assoc_data = {'assoc_var_name': assoc_var_name, 'expr_var_name': expr_var_name}
        self.assoc.append(assoc_data)

    def add_property(self, prop_global_var, comparisons, _type, line):
        property_data = {'prop_global_var': prop_global_var, 'comparisons': comparisons, 'type': _type, 'line': line}
        self.properties.append(property_data)
        self.property_counter +=1
    
    def get_prop_counter(self):
        return self.property_counter

    def get_library(self, msg_type):
        command = f"cd {self.filepath} | rosmsg show {msg_type}"
        return subprocess.check_output(command, shell=True).decode("utf-8").split('\n')[0].split('[')[1].split('/')[0]

    def get_code(self):
        file_loader = FileSystemLoader('templates')
        env = Environment(loader=file_loader,extensions=['jinja2.ext.do'])
        template = env.get_template('program.jinja')
        return template.render(file_prefix=self.file_prefix, sim_sub=self.sim_subscriber, 
                               subscribers=self.subscribers, var_list=self.vars,
                               properties=self.properties)

def added_var(name, _vars):
    for entry in _vars:
        if entry['name'] == name:
            return True
    return False

def sim_funcs(_object, func, args, ctx):
    """ Update the context depending on the function used """
    var_name,extract = None,None
    if func == 'position':
        args = ['position'] + args
        var_name = _object + '_' + '_'.join(args) + '_var_sim'
        extract = 'model_states_msg.pose[model_states_indexes[\'' + _object + '\']].' + '.'.join(args)
    elif func == 'velocity':
        var_name = _object + '_velocity_' + '_'.join(args) + '_var_sim'
        if args == []:
            extract = '(model_states_msg.twist[model_states_indexes[\'' + _object + '\']].linear.x**2 + model_states_msg.twist[model_states_indexes[\'' + _object + '\']].linear.y**2 + model_states_msg.twist[model_states_indexes[\'' + _object + '\']].linear.z**2' + ')**(1/2)'
        else:
            extract = 'model_states_msg.twist[model_states_indexes[\'' + _object + '\']].' + '.'.join(args)
    ctx.add_var(var_name, extract)
    return 'states[0][\'' + var_name + '\']'

prefixes = {'': '', 'always': 'not ', 'after': '', 'never': '', 'until': 'not '}
def prop_prefix(_property):
    return prefixes[_property]

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
    'after_until' : 'AFTER_UNTIL',
    'implies' : 'IMPLIES',
    'not' : 'NOT',
    'and' : 'AND',
    'or' : 'OR'
}

literals = ['>','<','(',')','+','-','*','/','{','}','@','=',':',',','.',';']

tokens = ['NAME','TOPIC_NAME','INTEGER','FLOAT','EQ','DIF','GTE',
         'LEE','FUNC_MAIN'] + list(reserved.values())