# labcommand_msgs/RequestSync: the Mac asks the Spark to resend what it missed on one topic.
# Mac -> Spark. The Spark answers with the missed messages or a snapshot on the topic's sync topic,
# then one SyncEnd on /room_memory/sync/end.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 5).

uint8 TOPIC_MAP_BRICKS=0          # /room_memory/map_bricks
uint8 TOPIC_EVENTS=1              # /room_memory/events
uint8 TOPIC_EVENT_ARCHIVE=2       # one generation's durable events; not the live stream (D47)

uint64 SINCE_NONE=18446744073709551615   # since_epoch and since_seq when the Mac holds nothing for the topic
uint64 GENERATION_UNKNOWN=0      # expected_generation when the Mac's since belongs to no known generation

# Common fields, first in every labcommand_msgs message.
string room_id                    # room to resynchronize
uint64 map_epoch                  # repeats since_epoch
uint64 seq                        # this client's request counter, +1 per request, starts at 0
builtin_interfaces/Time stamp     # when the Mac sent the request

uint64 since_epoch                # map_epoch of the message at the Mac's baseline for the topic (section 5), or SINCE_NONE
uint64 since_seq                  # seq of that message, or SINCE_NONE; both SINCE_NONE or neither
uint8 topic                       # TOPIC_*
uint64 expected_generation       # publisher generation the since belongs to; GENERATION_UNKNOWN = none. since is
                                 # honoured only when it equals the node's generation
uint8[16] request_token          # random 128-bit token, new for every send (retries too), copied into the SyncEnd; never all zero
