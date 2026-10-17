"""Protocol script. Planned watches may be shorter; protocol durations may not.

The detector diffs against a frozen reference, not the previous frame. Removing the
object that was just placed restores the reference and expires the appearance. It does
not produce DISAPPEARED. Removal and the move therefore use two reference objects,
and placement uses a third object that is not in the reference.
"""

PROTOCOL = (
    {
        "id": "baseline_idle",
        "class": "idle",
        "kind": None,
        "protocol_s": 60,
        "prompt": "Clear the room back to the reference. Press Enter when it matches the reference and nobody is in view.",
    },
    {
        "id": "box_placed",
        "class": "change",
        "kind": "appeared",
        "protocol_s": 120,
        "prompt": "Place a NEW object on a spot that is empty in the reference. Do not move the reference objects.",
    },
    {
        "id": "box_removed",
        "class": "change",
        "kind": "disappeared",
        "protocol_s": 120,
        "prompt": "Remove the reference object you named for removal. Do not remove the object you just placed.",
    },
    {
        "id": "box_moved",
        "class": "change",
        "kind": "moved",
        "protocol_s": 120,
        "prompt": "Move the other reference object about 1 m, and stay under 1.5 m. Leave the placed object where it is.",
    },
    {
        "id": "person_walk",
        "class": "activity",
        "kind": None,
        "protocol_s": 105,
        "prompt": "Walk through the room and leave. The watch includes time after you leave so a late candidate stays inside it.",
    },
    {
        "id": "person_sit",
        "class": "activity",
        "kind": None,
        "protocol_s": 120,
        "prompt": "Sit still in view. A still person can be promoted after about 40 s; this watch is the full two minutes.",
    },
    {
        "id": "restore_reference",
        "class": "restoration",
        "kind": None,
        "protocol_s": 60,
        "prompt": "Put every reference object back and remove the placed object. This interval is not an expected detection. Type skip to leave the scene as it is.",
    },
    {
        "id": "hour_idle",
        "class": "idle",
        "kind": None,
        "protocol_s": 3600,
        "prompt": "Leave the room static. The protocol watch is one hour. A shorter watch cannot be full G5 acceptance.",
    },
)

REFERENCE_PROMPT = """\
The detector compares each window to the frozen reference, not the previous frame.
TRACKING does not prove the reference contains the right objects, and a saved reference is not checked by this tool.
This tool does not restart the node and does not change parameters.

You need three distinct objects:
  PLACE: a new object that is not in the reference. That spot is empty in the reference.
  REMOVE: a different object that is already in the reference.
  MOVE: a third object that is already in the reference. You will move it about 1 m.
Do not remove the object you place.

Type verified if you have checked the reference itself.
Type incompatible if you cannot verify it, it is empty, or you would remove the object you place.
Type abort to stop.
"""

TOKEN_LINE = "new_object reference_object other_reference_object"


def resolve_durations(args):
    """``--watch-s`` changes baseline and the change watches only. Sit, walk, and the hour stay at protocol unless their own flag is set."""
    watch = getattr(args, "watch_s", None)
    return {
        "baseline_idle": 60 if watch is None else watch,
        "change": 120 if watch is None else watch,
        "person_walk": 105 if getattr(args, "walk_s", None) is None else args.walk_s,
        "person_sit": 120 if getattr(args, "sit_s", None) is None else args.sit_s,
        "hour_idle": 3600 if getattr(args, "hour_s", None) is None else args.hour_s,
        "restore_reference": 60,
    }
