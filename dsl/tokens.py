""" The language possible tokens """

reserved = {
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
    'or' : 'OR',
    'position_x' : 'POSITION_X',
    'position_y' : 'POSITION_Y',
    'position_z' : 'POSITION_Z',
    'velocity' : 'VELOCITY',
    'distance' : 'DISTANCE',
    'localization_error' : 'LOCALIZATION_ERROR',
    'orientation' : 'ORIENTATION'
}

literals = ['>','<','(',')','+','-','*','/','{','}','@','=',':',',','.']

tokens = ['NAME','TOPIC_NAME','INTEGER','FLOAT','EQ','DIF','GTE','LEE'] + list(reserved.values())