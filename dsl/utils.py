""" A set of usefull classes for the whole compiler """

class Context(object):
    """ Save the context of a program """
    def __init__(self):
        self.stack = [{}]

class Emitter(object):
    """ Save the strings to be written in the monitor file """
    def __init__(self):
        self.count = 0
        self.lines = []

    def get_count(self):
        self.count += 1
        return self.count

    def get_id(self):
        id = self.get_count()
        return f"var{id}"

    def __lshift__(self, v):
        self.lines.append(v)

    def get_code(self):
        return "\n".join(self.lines)

class Node(object):
    """ The ast of a program """
    def __init__(self, t, *args):
        self.type = t
        self.args = args

    def __str__(self):
        s = "rule: " + str(self.type) + "\n"
        s += "".join(["i: " + str(i) + "\n" for i in self.args])
        return s