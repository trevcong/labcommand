# labcommand_msgs/OperatorAction: one operator action on the room memory, sent from the Mac.
# Mac -> Spark as the request of the /room_memory/operator/action service (OperatorAction.srv, the
# :8767 bridge's services capability), or on /room_memory/operator from a relay. The node has both
# only with operator_actions:=true. The service response only acknowledges; the effect is the
# Spark's next ChangeEvent or MapBricks.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 3.6, D46).

uint8 ACTION_ACKNOWLEDGE=0        # event: the operator has seen it; logged, no state change
uint8 ACTION_ACCEPT=1             # CONFIRMED event -> ACCEPTED; the reference absorbs it, map_epoch bumps
uint8 ACTION_DISMISS=2            # CANDIDATE or CONFIRMED event -> REVERTED ("operator dismiss"); the change is suppressed
uint8 ACTION_LABEL=3              # event: store text as the label of its region; not implemented (UNSUPPORTED)
uint8 ACTION_MARK_GROUND_TRUTH=4  # event or none: ground-truth mark at stamp with text; not implemented
uint8 ACTION_PAUSE=5              # room: any -> PAUSED; not implemented
uint8 ACTION_RESUME=6             # room: PAUSED -> the state the guard check gives; not implemented
uint8 ACTION_REREFERENCE=7        # room: re-baseline, RIG_MOVED -> BUILDING; not implemented
uint8 ACTION_CONFIRM=8            # CANDIDATE event -> CONFIRMED ("operator confirm")

uint64 EVENT_NONE=18446744073709551615   # event_id of a room-level action
uint8 STATE_NONE=255                     # expected_state when event_id is EVENT_NONE

# Common fields, first in every labcommand_msgs message. They are this message's header.
string room_id                    # room the action applies to
uint64 map_epoch                  # epoch of the map the operator was shown; ACCEPT is compare-and-set on the published one
uint64 seq                        # this client's action counter, +1 per action, starts at 0
builtin_interfaces/Time stamp     # when the operator acted; never {0, 0}

uint8 action                      # ACTION_*
uint64 event_id                   # target ChangeEvent.event_id, or EVENT_NONE
uint8 expected_state              # ChangeEvent.STATE_* the operator saw, or STATE_NONE; CONFIRM needs CANDIDATE, ACCEPT CONFIRMED, DISMISS either
uint64 expected_generation        # the publisher_generation the operator saw; the node refuses any other; never 0
string text                       # label (LABEL) or description (MARK_GROUND_TRUTH), else an optional note; "" = none; <= 256 bytes
string operator_id                # who the operator says they are; non-empty, <= 64 bytes; self-asserted, not authenticated
string action_id                  # client-generated UUID, 36 lowercase characters; a repeat returns the first outcome
