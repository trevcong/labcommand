# labcommand_msgs/RequestSync: the Mac asks the Spark to resend what it missed on one topic.
# Mac -> Spark. The Spark answers with the missed messages or a snapshot on the topic's sync topic.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 5).

uint8 TOPIC_MAP_BRICKS=0          # /room_memory/map_bricks
uint8 TOPIC_EVENTS=1              # /room_memory/events

uint64 SINCE_NONE=18446744073709551615   # since_epoch and since_seq when the Mac holds nothing for the topic

# Common fields, first in every labcommand_msgs message.
string room_id                    # room to resynchronize
uint64 map_epoch                  # repeats since_epoch
uint64 seq                        # this client's request counter, +1 per request, starts at 0
builtin_interfaces/Time stamp     # when the Mac sent the request

uint64 since_epoch                # map_epoch of the last message the Mac applied on the topic, or SINCE_NONE
uint64 since_seq                  # seq of that message, or SINCE_NONE; both SINCE_NONE or neither
uint8 topic                       # TOPIC_*
