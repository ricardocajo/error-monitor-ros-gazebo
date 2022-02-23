""" The language grammar """

from utils import *

precedence = (
    ('nonassoc','GTE','LEE','EQ','DIF','>','<'),
    ('left','+','-'),
    ('left','/','*'),
    ('left','{')
)
#TODO shift-reduce conflict and non-assoc not working
def p_program(p):
    '''program : command
               | command program'''
    if len(p) > 2:
        p[0] = Node('program', p[1], p[2])
    else:
        p[0] = Node('program', p[1])
    print('teste1')
    print(p[0])

def p_command(p):
    '''command : declaration
               | model
               | pattern
               | association'''
    p[0] = Node('command', p[1])
    print('teste2')
    print(p[0])

def p_declaration(p):
    '''declaration : DECL NAME TOPIC_NAME msgtype
                   | DECL NAME NAME msgtype'''
    p[0] = Node('declaration', p[2], p[3], p[4])
    print('teste3')
    print(p[0])

def p_model(p):
    '''model : MODEL NAME ':' NAME TOPIC_NAME msgtype
             | MODEL NAME ':' NAME NAME msgtype'''
    p[0] = Node('model', p[2], p[4], p[5], p[6])
    print('teste4')
    print(p[0])

def p_msgtype(p):
    '''msgtype : NAME
               | NAME '.' msgtype'''
    if len(p) > 2:
        p[0] = Node('msgtype', p[1], p[3])
    else:
        p[0] = Node('msgtype', p[1])
    print('teste5')
    print(p[0])

def p_association(p):
    '''association : NAME '=' expression'''
    p[0] = Node('association', p[1], p[3])
    print('teste6')
    print(p[0])

def p_expression(p):
    '''expression : operation
                  | comparison
                  | operand'''
    p[0] = Node('expression', p[1])
    print('teste7')
    print(p[0])

def p_pattern_4(p):
    '''pattern : ALWAYS '(' paargs ')'
               | NEVER '(' paargs ')'
               | EVENTUALLY '(' paargs ')'
               | NOT '(' paargs ')' '''
    p[0] = Node('property', p[1], p[3])
    print('teste8')
    print(p[0])

def p_pattern_6(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' '''
    p[0] = Node('property', p[1], p[3], p[5])
    print('teste9')
    print(p[0])

def p_pattern_7(p):
    '''pattern : '(' paargs ')' UNTIL '(' paargs ')'
               | '(' paargs ')' IMPLIES '(' paargs ')' '''
    p[0] = Node('property', p[2], p[4], p[6])
    print('teste10')
    print(p[0])

def p_pattern_10(p):
    '''pattern : AFTER '(' paargs ',' paargs ')' UNTIL '(' paargs ')' '''
    p[0] = Node('property', p[1], p[3], p[5], p[7], p[9])
    print('teste11')
    print(p[0])

def p_patter_multi(p):
    '''pattern : '(' paargs ')' AND '(' paargs ')'
               | '(' paargs ')' OR '(' paargs ')'
               | '(' paargs ')' AND '(' paargs ')' pattsup
               | '(' paargs ')' OR '(' paargs ')' pattsup'''
    if len(p) > 8:
        p[0] = Node('pattern_multi', p[2], p[4], p[6], p[8])
    else:
        p[0] = Node('pattern_multi', p[2], p[4], p[6])
    print('teste12')
    print(p[0])

def p_pattsup(p):
    '''pattsup : AND '(' paargs ')' 
               | OR '(' paargs ')' 
               | AND '(' paargs ')' pattsup 
               | OR '(' paargs ')' pattsup'''
    if len(p) > 5:
        p[0] = Node('pattsup', p[1], p[3], p[5])
    else:
        p[0] = Node('pattsup', p[1], p[3])
    print('teste13')
    print(p[0])

def p_paargs(p):
    '''paargs : pattern
              | comparison
              | NAME'''
    p[0] = Node('paargs', p[1])
    print('teste14')
    print(p[0])

def p_operation(p):
    '''operation : expression '+' operand
                 | expression '-' operand
                 | expression '/' operand
                 | expression '*' operand'''
    p[0] = Node('operation', p[1], p[2], p[3]) 
    print('teste15')
    print(p[0])      

def p_comparison(p):
    '''comparison : expression opbin expression
                  | expression opbin '{' number '}' expression'''
    if len(p) > 4:
        p[0] = Node('comparison', p[1], p[2], p[6], p[4])
    else:
        p[0] = Node('comparison', p[1], p[2], p[3])
    print('teste16')
    print(p[0])

def p_opbin(p):
    '''opbin : '<'
             | '>'
             | GTE
             | LEE
             | EQ
             | DIF'''
    p[0] = Node('opbin', p[1])
    print('teste17')
    print(p[0])

def p_number(p):
    '''number : FLOAT
              | INTEGER'''
    p[0] = Node('number', p[1])
    print('teste18')
    print(p[0])

def p_operand(p):
    '''operand : func
               | number
               | NAME
               | bool
               | temporalvalue'''
    p[0] = Node('operand', p[1])
    print('teste19')
    print(p[0])

def p_bool(p):
    '''bool : TRUE
            | FALSE'''
    p[0] = Node('bool', p[1])
    print('teste20')
    print(p[0])

def p_func(p):
    '''func : NAME '.' FUNC_MAIN
            | NAME '.' FUNC_MAIN funcargs'''
    if len(p) > 4:
        p[0] = Node('func', p[1], p[3], p[4])
    else:
        p[0] = Node('func', p[1], p[3])
    print('teste21')
    print(p[0])

def p_funcargs(p):
    '''funcargs : '.' NAME
                | '.' NAME funcargs'''
    if len(p) > 3:
        p[0] = Node('funcargs', p[2], p[3])
    else:
        p[0] = Node('funcargs', p[2])
    print('teste22')
    print(p[0])

def p_temporalvalue(p):
    '''temporalvalue : '@' '{' NAME ',' INTEGER '}' '''
    p[0] = Node('temporalvalue',p[3], p[5])
    print('teste23')
    print(p[0])

def p_error(p):
    print("Syntax error at '%s'. Line number '%d'" % (p.value, p.lineno))