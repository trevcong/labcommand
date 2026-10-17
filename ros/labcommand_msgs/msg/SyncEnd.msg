# labcommand_msgs/SyncEnd: the end-of-answer manifest of one RequestSync. Spark -> Mac on
# /room_memory/sync/end, published after the answer's messages. It travels on its own topic, so it
# may arrive before them: it says what was sent, never that it was received.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo (sections 3.8 and 5).

uint64 SEQ_NONE=18446744073709551615     # bound when nothing was published on the topic in this generation

# Common fields, first in every labcommand_msgs message.
string room_id
uint64 map_epoch                  # the node's published epoch when the answer was computed
uint64 seq                        # +1 per SyncEnd this process publishes, from 0. An archive answer does not restart it, and the value is not a seq of the archived generation
builtin_interfaces/Time stamp

uint64 publisher_generation       # the generation the answer's messages carry. Live: this process. Archive: the named generation. Never 0
uint8[16] request_token           # copied from the RequestSync; all zero for a legacy request
uint8 topic                       # RequestSync.TOPIC_* copied
uint64 bound                      # live: the topic's newest seq in this process. Archive: the named generation's last published seq. SEQ_NONE: nothing published
SeqRange[] delivered              # seqs published in this answer on the topic's sync topic; canonical, each <= bound
SeqRange[] unavailable            # requested seqs no longer held (events and event archive); canonical, disjoint from delivered. Empty for a map answer
