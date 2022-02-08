""" A set of usefull classes for the whole compiler """

class Context(object):
    def __init__(self):
        self.stack = [{}]