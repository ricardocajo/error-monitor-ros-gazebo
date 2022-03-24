""" The language grammar """
from utils import *

def p_program_1(p):
    '''program : command'''
    p[0] = Node('program', p[1])

def p_program_2(p):
    '''program : command program'''
    p[0] = Node('program', p[1], p[2])
        
def p_command(p):
    '''command : association
               | declaration
               | model
               | pattern'''
    p[0] = Node('command', p[1])

def p_association(p):
    '''association : NAME '=' INTEGER'''  #TODO check what to do with this
    p[0] = Node('association', p[1], p[3])

def p_declaration(p):
    '''declaration : DECL NAME TOPIC_NAME msgtype
                   | DECL NAME NAME msgtype'''
    p[0] = Node('declaration', p[2], p[3], p[4])

def p_model(p):
    '''model : MODEL NAME ':' modelargs ';' '''
    p[0] = Node('model', p[2], p[4])

def p_modelargs_1(p):
    '''modelargs : NAME TOPIC_NAME msgtype
                 | NAME name msgtype'''
    p[0] = Node('modelargs', p[1], p[2], p[3])

def p_modelargs_2(p):
    '''modelargs : NAME TOPIC_NAME msgtype modelargs
                 | NAME name msgtype modelargs'''
    p[0] = Node('modelargs', p[1], p[2], p[3], p[4])

def p_msgtype_1(p):
    '''msgtype : name'''
    p[0] = Node('msgtype', p[1])

def p_msgtype_2(p):
    '''msgtype : name '.' msgtype'''
    p[0] = Node('msgtype', p[1], p[3])
        
def p_name(p):
    '''name : NAME
            | FUNC_MAIN'''
    p[0] = p[1]

def p_pattern_1(p):  #TODO check what to do with implies
    '''pattern : ALWAYS pattern 
               | NEVER pattern
               | EVENTUALLY pattern
               | IMPLIES pattern
               | NOT pattern'''
    p[0] = Node('property', p[1], p.lineno(1), p[2])

def p_pattern_2(p):
    '''pattern : AFTER pattern ',' pattern 
               | UNTIL pattern ',' pattern'''
    p[0] = Node('property', p[1], p.lineno(1), p[2], p[4])

def p_pattern_4(p):
    '''pattern : AFTER pattern ',' pattern UNTIL pattern'''
    p[0] = Node('property', p[1], p.lineno(1), p[5], p[2], p[4], p[6])

def p_pattern_0(p):
    '''pattern : conjunction'''
    p[0] = Node('property', p[1])

def p_conjunction(p):
    '''conjunction : conjunction AND comparison
                   | conjunction OR comparison'''
    p[0] = Node('conjunction', p[2], p[1], p[3])

def p_conjunction_0(p):
    '''conjunction : comparison'''
    p[0] = Node('conjunction', p[1])

def p_comparison_1(p):
    '''comparison : multiplication opbin multiplication'''
    p[0] = Node('comparison', p[2], p[1], p[3])

def p_comparison_2(p):
    '''comparison : multiplication opbin '{' number '}' multiplication'''
    p[0] = Node('comparison', p[2], p[1], p[6], p[4])

def p_comparison_0(p):
    '''comparison : multiplication'''
    p[0] = Node('multiplication', p[1])

def p_opbin(p):
    '''opbin : '<'
             | '>'
             | GTE
             | LEE
             | EQ
             | DIF'''
    p[0] = p[1]

def p_multiplication(p):
    '''multiplication : multiplication '/' addition
                      | multiplication '*' addition'''
    p[0] = Node('multiplication', p[2], p[1], p[3]) 

def p_multiplication_0(p):
    '''multiplication : addition'''
    p[0] = Node('multiplication', p[1])

def p_addition(p):
    '''addition : addition '+' operand
                | addition '-' operand'''
    p[0] = Node('addition', p[2], p[1], p[3])

def p_addition_0(p):
    '''addition : operand'''
    p[0] = Node('addition', p[1])

def p_operand(p):
    '''operand : NAME
               | number
               | TRUE
               | FALSE
               | func
               | temporalvalue'''
    p[0] = Node('operand', p[1])

def p_operand_par(p):
    '''operand : '(' pattern ')' '''
    p[0] = Node('operand', p[2])

def p_number(p):
    '''number : FLOAT
              | INTEGER'''
    p[0] = p[1]

def p_func_1(p):
    '''func : NAME '.' FUNC_MAIN funcargs'''
    p[0] = Node('func', p[1], p[3], p[4])

def p_func_2(p):
    '''func : NAME '.' FUNC_MAIN'''
    p[0] = Node('func', p[1], p[3])
        
def p_funcargs_1(p):
    '''funcargs : '.' name funcargs'''
    p[0] = Node('funcargs', p[2], p[3])

def p_funcargs_2(p):
    '''funcargs : '.' name'''
    p[0] = Node('funcargs', p[2])
        
def p_temporalvalue(p):
    '''temporalvalue : '@' '{' NAME ',' INTEGER '}' '''
    p[0] = Node('temporalvalue', p[3], p[5])

def p_error(p):
    print("Syntax error at '%s'. Line number '%d'" % (p.value, p.lineno))