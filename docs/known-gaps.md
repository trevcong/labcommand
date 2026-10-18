# Known gaps

Name objects on this branch is the protocol, the codecs, the registry library, and the Mac mirror on fixture snapshots. The wiring PR is not in this branch. Nothing here writes a reference, clears a redo on the live accept path, or publishes ACCEPTED.

These four items block that wiring PR. Each one lands in or before it, with the tests named here.

- **`save()` at startup and at BUILDING→TRACKING skips the durability marker.** Gate those saves, or send their failure into the same retry state as `advance_epoch`. The failure is swallowed in the current node, and it causes no harm until a redo depends on the marker.
  - Tests: a failed save during startup recovery publishes no map at that epoch and keeps the redo. The same holds for the first map after building.
- **`close_window` calls `commit()` without catching its exception.** That call is in #32 (`memory.cpp` at d5c0850). Define one failure path for both places that commit. It keeps the queued ACCEPTED, keeps the node running, and publishes it exactly once.
  - Tests: consecutive failures across both commit sites, then recovery in a window with no map. This needs #32's code.
- **What relearn does to accepts still waiting in `held_`.** The wiring PR calls the registry's frame-change function, and either stops publishing EXPIRED for a held accept or relies on receipt-wins. The library already keeps a receipt ahead of EXPIRED. The node does not call it yet.
  - Tests: relearn before the map is published, and relearn after a failed log commit. This needs #31's code.
- **Telling whether a redo already reached the reference.** The accept gate blocks a second accept while a redo is pending. The overlap that remains is a settle merge while the redo waits to be cleared. Replaying there misstates a few voxels. That shows up as a new candidate and is fixed through the operator flow, and no names are lost. The registry file is versioned, so an epoch field can be added later.

Also not in this branch: the node does not advertise `/room_memory/objects/name` or publish `/room_memory/objects` (both need `operator_actions`, `rig_guard`, and #31). The Mac does not subscribe, and it has no naming sheet and no timeline labels. The second-move tracker (paired, both halves, one half) stays in the memory tracker. The linker is tested on the event shapes that tracker produces.
