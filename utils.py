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
        self.subscribers = [] # 'topic','msgtype','library','sub_name'
        self.sim_subscriber = False
        self.vars = [] # 'name','object_name','args','sim'
        self.properties = [] # 'property',

    def add_subscriber(self, topic, msgtype, library, sub_name):
        sub_data = {'topic': topic, 'msgtype': msgtype, 'library': library, 'sub_name': sub_name}
        self.subscribers.append(sub_data)

    def add_sim_subscriber(self):
        if self.sim_subscriber is False:
            self.add_subscriber('/gazebo/model_states', 'ModelStates', 'gazebo_msgs', 'model_states_sub')
            self.sim_subscriber = True

    def add_var(self, name, object_name, sub, arg, arg_extra):
        var_data = {'name': name, 'object_name': object_name, 'sub': sub, 'arg': arg, 
                    'arg_extra': arg_extra}
        self.vars.append(var_data)

    def add_property(self, _property, comp1_var1, comp1_var2, op_bin1, comp2_var1=None, comp2_var2=None):
        property_data = {'property': _property, 'comp1_var1': comp1_var1, 'comp1_var2': comp1_var2, 
                        'op_bin1': op_bin1, 'comp2_var1': comp2_var1, 'comp2_var2': comp2_var2}
        self.properties.append(property_data)

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

def sim_funcs(_object, func, args, ctx):
    """ Update the context depending on the function used """
    if func == 'position':
        args = set(['position'] + (args or []))
        var_name = _object + '_pose_' + '_'.join(args) + '_var_sim'
        ctx.add_var(var_name, _object, 'position', 'pose', '.'.join(args))
    elif func == 'velocity':
        args = set(['x', 'linear'] + (args or []))
        var_name = _object + '_twist_' + '_'.join(args) + '_var_sim'
        ctx.add_var(var_name, _object, 'velocity', 'twist', '.'.join(args))
    return 'sim_state[0].get(\'' + var_name + '\')'

ops = {'<':lambda x,y:x+y}
#ops[op](left,right)
#abs(a-b) <= max( rel_tol * max(abs(a), abs(b)), abs_tol)

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