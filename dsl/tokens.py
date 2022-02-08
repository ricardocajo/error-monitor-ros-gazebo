reserved = {
    'always' : 'ALWAYS',
    'position_x' : 'POSITION_X',
    'position_y' : 'POSITION_Y',
    'and' : 'AND'
}

literals = ['>','(',')']

tokens = ['NAME','FLOAT'] + list(reserved.values())