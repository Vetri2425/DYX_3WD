import os
import re

from .msgs import BUILTINS


class Field:
    def __init__(self, type_, name, pkg):
        base = re.sub(r"\[.*\]$", "", type_)
        self.name = name
        self.is_header = base in ("Header", "std_msgs/Header")
        if base not in BUILTINS and "/" not in base:
            # genmsg resolves a bare nested type to <package>/<Type>, keeping any array suffix
            self.type = f"{pkg}/{type_}"
            self.base_type = f"{pkg}/{base}"
        else:
            self.type = type_
            self.base_type = base


class Spec:
    def __init__(self, fields):
        self._fields = fields

    def parsed_fields(self):
        return self._fields


class MsgContext:
    @staticmethod
    def create_default():
        return MsgContext()


def load_msg_by_type(ctx, full_type, search_path):
    pkg, name = full_type.split("/", 1)
    for d in search_path.get(pkg, []):
        path = os.path.join(d, name + ".msg")
        if os.path.exists(path):
            return Spec(parse(open(path).read(), pkg))
    raise FileNotFoundError(full_type)


def parse(text, pkg):
    fields = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" in line:  # blank or constant
            continue
        tok = line.split()
        fields.append(Field(tok[0], tok[1], pkg))
    return fields
