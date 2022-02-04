reserved = {
    'always' : 'ALWAYS',
    'position_x' : 'POSITION_X',
    'position_y' : 'POSITION_Y',
    'and' : 'AND'
}

literals = ['>','(',')']

tokens = ['NAME','FLOAT'] + list(reserved.values())

t_ignore = ' \t'
t_ignore_COMMENT = r'\#.*'

def t_NAME(t):
    r'[a-zA-Z_][a-zA-Z0-9_]*'
    t.type = reserved.get(t.value,'NAME')
    return t
    
def t_FLOAT(t):
    r'[-]?[0-9]*[.][0-9]+'
    try:
        t.value = float(t.value)
    except ValueError:
        print("Float value too large %d", t.value)
        t.value = 0
    return t
    
def t_newline(t):
    r'\n+'
    t.lexer.lineno += t.value.count("\n")

def t_error(t):
    print("Illegal character '%s'" % t.value[0])
    t.lexer.skip(1)