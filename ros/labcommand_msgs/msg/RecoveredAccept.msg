# labcommand_msgs/RecoveredAccept: a receipt for an accept that was answered APPLIED
# and never logged ACCEPTED. Contract: docs/ROOM-MEMORY-PROTOCOL.md section 3.15.
# A receipt may name any generation, including the snapshot's own publisher_generation.

uint8 KIND_APPEARED=0
uint8 KIND_DISAPPEARED=1
uint8 KIND_MOVED=2

uint64 event_generation          # never 0
uint64 event_id                  # never 0
uint8 kind                       # KIND_*
