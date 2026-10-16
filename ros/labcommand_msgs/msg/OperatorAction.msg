# labcommand_msgs/OperatorAction: one operator action on the room memory, sent from the Mac.
# Mac -> Spark on /room_memory/operator. No reply message: the outcome is the Spark's next
# ChangeEvent, MapBricks or MemoryStatus.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 3.6).
# A proposal, out of scope for protocol v2: no codec, golden or node handler yet (Phase 3 stage S5).

uint8 ACTION_ACKNOWLEDGE=0        # event: the operator has seen it; logged, no state change
uint8 ACTION_ACCEPT=1             # CONFIRMED event -> ACCEPTED; the reference absorbs it, map_epoch bumps
uint8 ACTION_DISMISS=2            # CONFIRMED event -> REVERTED ("operator revert"); reference unchanged
uint8 ACTION_LABEL=3              # event: store text as the label of its region; logged
uint8 ACTION_MARK_GROUND_TRUTH=4  # event or none: ground-truth mark at stamp with text, for scoring
uint8 ACTION_PAUSE=5              # room: any -> PAUSED
uint8 ACTION_RESUME=6             # room: PAUSED -> the state the guard check gives
uint8 ACTION_REREFERENCE=7        # room: re-baseline, RIG_MOVED -> BUILDING; the new reference bumps map_epoch

uint64 EVENT_NONE=18446744073709551615   # event_id of a room-level action
uint8 STATE_NONE=255                     # expected_state when event_id is EVENT_NONE

# Common fields, first in every labcommand_msgs message. They are this message's header.
string room_id                    # room the action applies to
uint64 map_epoch                  # epoch of the map the operator was shown; ACCEPT and REREFERENCE are compare-and-set on it
uint64 seq                        # this client's action counter, +1 per action, starts at 0
builtin_interfaces/Time stamp     # when the operator acted

uint8 action                      # ACTION_*
uint64 event_id                   # target ChangeEvent.event_id, or EVENT_NONE
uint8 expected_state              # ChangeEvent.STATE_* the operator saw, or STATE_NONE; ACCEPT and DISMISS require CONFIRMED
string text                       # label (LABEL) or description (MARK_GROUND_TRUTH), else an optional note; "" = none; <= 256 bytes
string operator_id                # who the operator says they are; non-empty, <= 64 bytes; self-asserted, not authenticated
string action_id                  # client-generated UUID, 36 lowercase characters; a repeat is ignored
