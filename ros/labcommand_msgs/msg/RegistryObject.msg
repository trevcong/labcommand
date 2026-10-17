# labcommand_msgs/RegistryObject: one named object in an ObjectRegistry snapshot.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md section 3.15.
# association is in the snapshot only. The registry file does not store it.

uint8 BASELINE_ANCHORED=0
uint8 BASELINE_GONE=1
uint8 BASELINE_DETACHED=2
uint8 BASELINE_RETIRED=3

uint8 ASSOCIATION_NONE=0
uint8 ASSOCIATION_PENDING=1
uint8 ASSOCIATION_UNRESOLVED=2

uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2

uint64 object_id
uint64 revision                  # +1 on every durable change of this object; Create lands at 1
string name                      # UTF-8, 1...64 bytes, no control characters
uint8 baseline                   # BASELINE_*
uint8 association                # ASSOCIATION_*; NONE writes the two association fields as 0
uint64 association_generation    # the open event's generation when association is not NONE; else 0
uint64 association_event_id      # the open event's id when association is not NONE; else 0
uint8 sensor                     # one SENSOR_* bit
uint64 frame_epoch               # the anchor's room frame; 0 when baseline is GONE (no anchor)
float32[3] centroid              # m; NaN x3 when baseline is GONE
float32[6] aabb                  # m; NaN x6 when baseline is GONE
ObjectLink[<=32] links           # accepted links, oldest first; at most 32
