""" A set of usefull classes and structures for the whole compiler """

from jinja2 import FileSystemLoader, Environment
import subprocess

class TypeCheckerContext(object):
    """ Save the context of a program (in the paradigm of the type_checker function)"""
    def __init__(self):
        self.topic_value = []
        self.assoc = []

    def add_topic_value(self, name):
        self.topic_value.append(name)

    def add_assoc(self, name):
        self.assoc.append(name)

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
        self.temporal_size = 1
        self.rate = 20
        self.timeout = 100
        self.eventually = 0

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

    def add_assoc(self, name, comparison_state):
        assoc_data = {'name': name, 'comparison_state': comparison_state}
        self.assoc.append(assoc_data)

    def is_assoc(self, name):
        return added_var(name, self.assoc)

    def assoc_info(self, name):
        for entry in self.assoc:
            if entry['name'] == name:
                return entry['comparison_state']

    def add_property(self, comparisons, line, type_):
        property_data = {'comparisons': comparisons, 'line': line, 'type': type_}
        self.properties.append(property_data)

    def get_library(self, msg_type):
        command = f"cd {self.filepath} | rosmsg show {msg_type}"
        return subprocess.check_output(command, shell=True).decode("utf-8").split('\n')[0].split('[')[1].split('/')[0]

    def check_temporal_size(self, value):
        if abs(value)+1 > self.temporal_size:
            self.temporal_size = abs(value)+1

    def rate_update(self, value):
        self.rate = value

    def timeout_update(self, value):
        self.timeout = value

    def get_code(self):
        file_loader = FileSystemLoader('templates')
        env = Environment(loader=file_loader,extensions=['jinja2.ext.do'])
        template = env.get_template('program.jinja')
        return template.render(file_prefix=self.file_prefix, sim_sub=self.sim_subscriber, 
                               subscribers=self.subscribers, var_list=self.vars,
                               properties=self.properties, temp_size=self.temporal_size,
                               rate=self.rate, timeout=self.timeout, eventually=self.eventually)

def added_var(name, list_):
    for entry in list_:
        if entry['name'] == name:
            return True
    return False

def sim_funcs(object_, func, args, ctx):
    """ Update the context depending on the function used """
    var_name,extract = None,None
    if func == 'position':
        args = ['position'] + args
        var_name = object_ + '_' + '_'.join(args) + '_var_sim'
        extract = 'model_states_msg.pose[model_states_indexes[\'' + object_ + '\']].' + '.'.join(args)
    elif func == 'velocity':
        var_name = object_ + '_velocity_' + '_'.join(args) + '_var_sim'
        if args == []:
            extract = '(model_states_msg.twist[model_states_indexes[\'' + object_ + '\']].linear.x**2 + model_states_msg.twist[model_states_indexes[\'' + object_ + '\']].linear.y**2 + model_states_msg.twist[model_states_indexes[\'' + object_ + '\']].linear.z**2' + ')**(1/2)'
        else:
            extract = 'model_states_msg.twist[model_states_indexes[\'' + object_ + '\']].' + '.'.join(args)
    ctx.add_var(var_name, extract)
    return 'states[0][\'' + var_name + '\']'

prefixes = {'': '', 'always': 'not ', 'after': '', 'never': '', 'until': 'not ', 'eventually': ''}
def prop_prefix(property_):
    return prefixes[property_]

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
    'and' : 'AND',
    'or' : 'OR',
    '_rate_' : 'RATE',
    '_timeout_' : 'TIMEOUT'
}

literals = ['>','<','(',')','+','-','*','/','{','}','@','=',':',',','.',';']

tokens = ['NAME','TOPIC_NAME','INTEGER','FLOAT','EQ','DIF','GTE',
         'LEE','FUNC_MAIN'] + list(reserved.values())