""" A set of usefull classes and structures for the whole compiler """
from __future__ import annotations

import subprocess

from jinja2 import Environment
from jinja2 import FileSystemLoader


class TypeCheckerContext:
    """Save the context of a program (in the paradigm of the type_checker function)"""

    def __init__(self):
        self.topic_value = []
        self.assoc = []
        self.model = []
        self.default_margin = False

    def add_topic_value(self, name):
        self.topic_value.append(name)

    def add_assoc(self, dic):
        self.assoc.append(dic)

    def add_model(self, object_, func):
        self.model.append({"object": object_, "func": func})

    def is_model(self, object_, func):
        return any(
            item["object"] == object_ and item["func"] == func for item in self.model
        )

    def is_topic_value(self, name):
        for value in self.topic_value:
            if value == name:
                return True
        return False

    def is_assoc(self, name):
        for dic in self.assoc:
            if dic["var"] == name:
                return True, dic["type"]
        return False, None

    def set_default_margin(self):
        self.default_margin = True

    def has_default_margin(self):
        return self.default_margin


class CompileContext:
    """Save the context of a program (in the paradigm of the compile_py function)"""

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
        self.rate = 30
        self.timeout = 100
        self.eventually = 0
        self.eventually_aux = 0
        self.is_after_or = False
        self.model = []
        self.default_margin = None

    def add_subscriber(self, topic, msgtype, library, sub_name):
        sub_data = {
            "topic": topic,
            "msgtype": msgtype,
            "library": library,
            "sub_name": sub_name,
        }
        self.subscribers.append(sub_data)

    def add_sim_subscriber(self):
        if self.sim_subscriber is False:
            self.add_subscriber(
                "/gazebo/model_states",
                "ModelStates",
                "gazebo_msgs",
                "model_states",
            )
            self.sim_subscriber = True

    def add_var(self, name, extract):
        if not added_var(name, self.vars):
            var_data = {"name": name, "extract": extract}
            self.vars.append(var_data)

    def add_assoc(self, name, comparison_state):
        assoc_data = {"name": name, "comparison_state": comparison_state}
        self.assoc.append(assoc_data)

    def is_assoc(self, name):
        return added_var(name, self.assoc)

    def assoc_info(self, name):
        for entry in self.assoc:
            if entry["name"] == name:
                return entry["comparison_state"]

    def add_property(self, comparisons, line, type_):
        property_data = {"comparisons": comparisons, "line": line, "type": type_}
        self.properties.append(property_data)

    def add_eventually(self):
        self.eventually += 1
        self.eventually_aux = 0
        return "var_" + str(self.eventually)

    def add_aux_eventually(self):
        self.eventually_aux += 1
        return (
            "var_" + str(self.eventually) + "aux_" + str(self.eventually_aux),
            self.eventually_aux,
        )

    def get_library(self, msg_type):
        command = f"cd {self.filepath} | rosmsg show {msg_type}"
        return (
            subprocess.check_output(command, shell=True)
            .decode("utf-8")
            .split("\n")[0]
            .split("[")[1]
            .split("/")[0]
        )

    def check_temporal_size(self, value):
        if abs(value) + 1 > self.temporal_size:
            self.temporal_size = abs(value) + 1

    def is_after_or_true(self):
        self.is_after_or = True

    def add_model(self, object_, func, msg_type):
        self.model.append({"object": object_, "func": func, "msg_type": msg_type})

    def model_msgtype(self, object_, func):
        return ".".join(
            next(
                item
                for item in self.model
                if item["object"] == object_ and item["func"] == func
            )["msg_type"][1:],
        )

    def rate_update(self, value):
        self.rate = value

    def timeout_update(self, value):
        self.timeout = value

    def default_margin_update(self, value):
        self.default_margin = value

    def get_default_margin(self):
        return self.default_margin

    def get_code(self):
        file_loader = FileSystemLoader("templates")
        env = Environment(loader=file_loader, extensions=["jinja2.ext.do"])
        template = env.get_template("program.jinja")
        return template.render(
            file_prefix=self.file_prefix,
            sim_sub=self.sim_subscriber,
            subscribers=self.subscribers,
            var_list=self.vars,
            properties=self.properties,
            temp_size=self.temporal_size,
            rate=self.rate,
            timeout=self.timeout,
            eventually=self.eventually,
            is_after_or=self.is_after_or,
        )


def added_var(name, list_):
    for entry in list_:
        if entry["name"] == name:
            return True
    return False


def sim_funcs(object_, func, args, ctx):
    """Update the context depending on the function used"""
    var_name, extract = None, None
    if func == "position":
        args = ["position"] + args
        var_name = object_ + "_" + "_".join(args) + "_var_sim"
        extract = (
            "model_states_msg.pose[model_states_indexes['"
            + object_
            + "']]."
            + ".".join(args)
        )
    elif func == "velocity":
        var_name = object_ + "_velocity_" + "_".join(args) + "_var_sim"
        if args == []:
            extract = (
                "(model_states_msg.twist[model_states_indexes['"
                + object_
                + "']].linear.x**2 + model_states_msg.twist[model_states_indexes['"
                + object_
                + "']].linear.y**2 + model_states_msg.twist[model_states_indexes['"
                + object_
                + "']].linear.z**2"
                + ")**(1/2)"
            )
        else:
            extract = (
                "model_states_msg.twist[model_states_indexes['"
                + object_
                + "']]."
                + ".".join(args)
            )
    elif func == "localization_error":
        var_name = object_ + "_localization_error"
        args = ctx.model_msgtype(object_, "position")
        extract = (
            "((model_states_msg.pose[model_states_indexes['"
            + object_
            + "']].position.x - "
            + object_
            + "_position_msg."
            + args
            + ".x)**2 + (model_states_msg.pose[model_states_indexes['"
            + object_
            + "']].position.y - "
            + object_
            + "_position_msg."
            + args
            + ".y)**2 + (model_states_msg.pose[model_states_indexes['"
            + object_
            + "']].position.z - "
            + object_
            + "_position_msg."
            + args
            + ".z)**2)**(1/2)"
        )
    ctx.add_var(var_name, extract)
    return "states[0]['" + var_name + "']"


prefixes = {
    "": "",
    "always": "not ",
    "after": "",
    "never": "",
    "until": "not ",
    "eventually": "",
}


def prop_prefix(property_):
    return prefixes[property_]


class Node:
    """The ast of a program"""

    def __init__(self, t, *args):
        self.type = t
        self.args = args

    def __str__(self):
        s = "rule: " + str(self.type) + "\n"
        s += "".join(["i: " + str(i) + "\n" for i in self.args])
        return s


# The default functions of the language

funcs = ["position", "velocity", "distance", "localization_error", "orientation"]

# The tokens of the language
reserved = {
    "true": "TRUE",
    "false": "FALSE",
    "decl": "DECL",
    "model": "MODEL",
    "always": "ALWAYS",
    "never": "NEVER",
    "eventually": "EVENTUALLY",
    "after": "AFTER",
    "until": "UNTIL",
    "after_until": "AFTER_UNTIL",
    "implies": "IMPLIES",
    "and": "AND",
    "or": "OR",
    "_rate_": "RATE",
    "_timeout_": "TIMEOUT",
    "_default_margin_": "DEFAULT_MARGIN",
    "position": "POSITION",
    "velocity": "VELOCITY",
    "distance": "DISTANCE",
    "localization_error": "LOCALIZATION_ERROR",
    "orientation": "ORIENTATION",
}

literals = [
    ">",
    "<",
    "(",
    ")",
    "+",
    "-",
    "*",
    "/",
    "{",
    "}",
    "@",
    "=",
    ":",
    ",",
    ".",
    ";",
]

tokens = ["NAME", "TOPIC_NAME", "INTEGER", "FLOAT", "EQ", "DIF", "GTE", "LEE"] + list(
    reserved.values(),
)
