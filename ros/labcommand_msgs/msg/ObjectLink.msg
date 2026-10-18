# labcommand_msgs/ObjectLink: one accepted link from an object to an event.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md section 3.15. Links are keyed by (event_generation, event_id).

uint8 KIND_APPEARED=0
uint8 KIND_DISAPPEARED=1
uint8 KIND_MOVED=2

uint64 event_generation          # the publishing generation of the linked event; never 0
uint64 event_id                  # ChangeEvent.event_id; never 0
uint8 kind                       # KIND_*, the event's kind
bool recovered                   # 1: the accept was answered APPLIED and no ACCEPTED was logged
