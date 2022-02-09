""" The language possible tokens """

reserved = {
    'always' : 'ALWAYS',
    'eventually' : 'EVENTUALLY',
    'until' : 'UNTIL',
    'implies' : 'IMPLIES',
    'not' : 'NOT',
    'and' : 'AND',
    'or' : 'OR',
    'position_x' : 'POSITION_X',
    'position_y' : 'POSITION_Y',
    'velocity' : 'VELOCITY',
    'distance' : 'DISTANCE',
    'localization_error' : 'LOCALIZATION_ERROR',
    'orientation' : 'ORIENTATION'
}

literals = ['>','<','(',')','+','-','*','/']

tokens = ['NAME','FLOAT','EQ','DIF','GTE','LEE'] + list(reserved.values())