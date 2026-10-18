# labcommand_msgs/ObjectRegistry: the whole name registry, one snapshot.
# Spark -> Mac on /room_memory/objects. Reliable, transient_local, keep-last 1.
# Contract: docs/ROOM-MEMORY-PROTOCOL.md section 3.15 (D51).
# v2 only: there is no legacy layout. publisher_generation is a required field, not a detected tail.
# ChangeEvent is unchanged. This message never decides a connection's layout.

string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp

uint64 snapshot_seq              # +1 on every durable or association change within one publisher generation
RegistryObject[] objects
NameableSeed[<=64] seeds
string[<=1024] recent_action_ids # APPLIED NameObject action ids still in the dedupe ring, oldest first
RecoveredAccept[<=64] recovered_accepts
uint64 publisher_generation      # section 1.4; never 0
