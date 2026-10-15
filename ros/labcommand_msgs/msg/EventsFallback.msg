# labcommand_msgs/EventsFallback: header of an events fallback, the Spark's answer to a TOPIC_EVENTS
# RequestSync it cannot replay. Spark -> Mac on /room_memory/sync/events_fallback, right before the
# fallback's ChangeEvents on /room_memory/sync/events. It names every events seq the fallback sends.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (section 3.8, D27).

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch                  # the node's published epoch when the fallback is sent
uint64 seq                        # +1 per header on this topic, starts at 0
builtin_interfaces/Time stamp

uint64 since_epoch                # copied from the RequestSync this fallback answers (RequestSync.SINCE_NONE copied too)
uint64 since_seq                  # copied likewise; both SINCE_NONE or neither
uint64 kept_first_seq             # seq of the oldest kept events message; <= kept_last_seq
uint64 kept_last_seq              # seq of the newest kept events message; every seq from kept_first_seq to here is sent
uint64[] resent_seqs              # seqs of open events' latest messages sent before the kept ones; increasing, each < kept_first_seq
