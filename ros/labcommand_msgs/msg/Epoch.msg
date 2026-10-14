# labcommand_msgs/Epoch: timeline tick, 0.2 Hz. Lets the Mac draw the timeline without the map.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md in the LabCommand repo.

# Common fields, first in every labcommand_msgs message. map_epoch is the epoch id.
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint32 n_appeared                 # APPEARED events that reached CONFIRMED since the room was created
uint32 n_disappeared              # DISAPPEARED events that reached CONFIRMED since the room was created
uint32 n_moved                    # MOVED events that reached CONFIRMED since the room was created
float32 coverage_known_frac       # 0..1, as in MemoryStatus; NaN = room volume not yet known
uint8 state                       # MemoryStatus STATE_* value at this tick
