"""One validation finding."""

from __future__ import annotations


class Issue(object):
    def __init__(self, level, path, message):
        self.level = level
        self.path = path
        self.message = message

    def format(self):
        where = self.path if self.path else "(spec)"
        return "{0}: {1}: {2}".format(self.level.upper(), where, self.message)

    def __str__(self):
        return self.format()


def errors_of(issues):
    return [item for item in issues if item.level == "error"]


def warnings_of(issues):
    return [item for item in issues if item.level == "warning"]
