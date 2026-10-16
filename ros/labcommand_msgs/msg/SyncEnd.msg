# labcommand_msgs/SyncEnd: the end-of-answer manifest of one RequestSync. Spark -> Mac on
# /room_memory/sync/end, published after the answer's messages. It travels on its own topic, so it
# may arrive before them: it says what was sent, never that it was received.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (sections 3.8 and 5).

uint64 SEQ_NONE=18446744073709551615     # bound when nothing was published on the topic in this generation

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch                  # the node's published epoch when the answer was computed
uint64 seq                        # +1 per SyncEnd, starts at 0 per publisher generation
builtin_interfaces/Time stamp

uint64 publisher_generation       # the answering generation: every message of the answer carries it; never 0
uint8[16] request_token           # copied from the RequestSync; all zero for a legacy request
uint8 topic                       # RequestSync.TOPIC_* copied
uint64 bound                      # the topic's newest live seq in this generation when the answer was computed, or SEQ_NONE
SeqRange[] delivered              # seqs published in this answer on the topic's sync topic; canonical, each <= bound
SeqRange[] unavailable            # requested seqs the publisher no longer holds (events only); canonical, disjoint from delivered
