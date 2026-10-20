"""Finding records and the two failure types the CLI maps to exit codes."""

from __future__ import annotations


# Exact M3 sentence. Case, spacing, and the semicolon are significant.
REPLAY_LABEL = "replay demonstrated; physical accuracy not evaluated"

REQUIRED_KEYS = (
    "run_id",
    "source_recording",
    "sensor",
    "fastlio_revision",
    "config_file",
    "config_sha256",
    "image",
    "created_at",
    "notes",
    "physical_accuracy_evaluated",
)

# Optional keys this package and the schema already know about.
# Anything else is an M2 warning, never an error.
KNOWN_KEYS = frozenset(
    REQUIRED_KEYS
    + (
        "accuracy_label",
        "files",
        "source_fingerprint",
    )
)

STRING_KEYS = (
    "run_id",
    "source_recording",
    "sensor",
    "fastlio_revision",
    "config_file",
    "config_sha256",
    "image",
    "created_at",
    "notes",
)


class Finding(object):
    """One validator result.

    severity is ``ERROR`` or ``WARNING``. rule is a short id such as ``M1``.
    """

    def __init__(self, rule, severity, message):
        self.rule = rule
        self.severity = severity
        self.message = message

    def __str__(self):
        return "%s %s %s" % (self.severity, self.rule, self.message)

    def __repr__(self):
        return "Finding(%r, %r, %r)" % (self.rule, self.severity, self.message)

    def to_dict(self):
        return {
            "severity": self.severity,
            "rule": self.rule,
            "message": self.message,
        }


class ManifestBuildError(Exception):
    """The builder refused to write. The CLI exits 1."""


class ManifestUnreadable(Exception):
    """The session input could not be read. The CLI exits 2."""


def error(rule, message):
    return Finding(rule, "ERROR", message)


def warning(rule, message):
    return Finding(rule, "WARNING", message)
