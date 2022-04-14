""" The lex tokenizer to split the language tokens """
from __future__ import annotations

from ply.lex import TOKEN

from utils import *

t_EQ = r"=="
t_DIF = r"!="
t_GTE = r">="
t_LEE = r"<="

t_ignore = " \t"
t_ignore_COMMENT = r"\#.*"


def t_NAME(t):
    r"[a-zA-Z_][a-zA-Z0-9_]*"
    t.type = reserved.get(t.value, "NAME")
    return t


def t_TOPIC_NAME(t):
    r"[a-zA-Z~/][a-zA-Z0-9_/]*"
    t.type = reserved.get(t.value, "TOPIC_NAME")
    return t


def t_FLOAT(t):
    r"[-]?[0-9]*[.][0-9]+"
    try:
        t.value = float(t.value)
    except ValueError:
        print(f"Float value too large {t.value}")
        t.value = 0
    return t


def t_INTEGER(t):
    r"[-]?[0-9][0-9_]*(?<!_)"
    try:
        t.value = int(t.value)
    except ValueError:
        print(f"Integer value too large {t.value}")
        t.value = 0
    return t


def t_newline(t):
    r"\n+"
    t.lexer.lineno += t.value.count("\n")


def t_error(t):
    print(f"Illegal character '{t.value[0]}'")
    t.lexer.skip(1)
