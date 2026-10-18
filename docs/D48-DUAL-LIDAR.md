# D48 — JT128 and Mid-360 in one room frame

Offline only. This change adds the frame contract and a calibration tool. It does not edit the Swift app, the C++ node, or the Python oracle, and it does not deploy anything. Until a later node applies the transforms below, D30 remains the wire rule: each sensor's memory stays in that sensor's frame, and the one published map is the map sensor's (the JT128 when it is configured).

The normative statement is protocol §3.12. This file is the method, the storage rule, and the measurements.

## Frames

Two transforms, not one.

`T_jt128←mid360` is the rigid mount this tool estimates:

```
p_jt128 = R p_livox + t
```

Parent frame `hesai_lidar`, child frame `livox_frame`. `R = Rz(yaw) Ry(pitch) Rx(roll)`, the same order as `RigidTransform` in `Sources/LabCommandCore/RigTransforms.swift`.

`T_room←jt128` is not this mount and is not estimated here. Protocol §1.2 and phase 3 P14 (`docs/phase3-live-integration.md` §4.9) require the room frame to be levelled so z is up. `RigTransforms.hesaiToWorld` is the identity. That identity is the Mac's drawing convention for the JT128 origin. It is not a gravity alignment, and this decision does not treat `hesai_lidar` as the room frame.

```
T_room←mid360 = T_room←jt128 × T_jt128←mid360
```

The nominal mount is the seed, not the calibrated value: yaw −86.5°, pitch 0, roll π, translation `(0, 0, −0.110)`. `mount()` in `spark/room_memory_cpp/src/memory.cpp` (lines 128–134) copies only the rotation of that mount, drops the 110 mm, and the comment says it is a rotation prior for the rig guard, never an extrinsic. `RigPose.mid360_pose` is a different object again: the guard's pose of the Mid-360 into its own reference (`p_ref = R p + t`, D40), identity when that reference is built.

## Method

Coarse init, then point-to-plane ICP. NumPy only. The tool does not import `room_memory.guard` and it does not use a global-registration library.

The seed is the nominal mount, or a `--init` file of schema `labcommand.extrinsic.v1`. `--init` is checked before any solve: schema, parent `hesai_lidar` and child `livox_frame`, convention `p_parent = R * p_child + t`, a finite 4×4, bottom row `0 0 0 1`, `R` orthonormal with det +1, and `translation_m` / `quaternion_xyzw` matching the matrix when they are present. A bad file exits 1 and writes nothing. The JSON records the matrix that was actually used as `T_init`, not only the name of the seed.

The scene has to be a stationary rig and a static room. The tool cannot see that from one pair, and it records the precondition instead of inferring it. The default range gate, 1.5 m to 8 m from each sensor, is not a static-structure test: a person or a moved object at 2 m stays inside it. Two further guards limit, and do not remove, that risk:

- Solver residuals are trimmed at `min(gate, max(0.05 m, 2.5 × median |r|))`, then Huber-weighted with δ = 0.05 m.
- Acceptance also requires the motion to stay inside 5 cm and 3° of the seed.

A mover that both sensors see, consistently, inside the band, can still pull an unweighted or a lightly weighted fit. Trimming does not replace a static recording.

Each cloud is range-gated, then voxel-downsampled at 5 cm (first point per cell, cap 20,000). Target normals come from the covariance of neighbors inside 0.20 m, at least 8 points, kept when the smallest eigenvalue is at most 0.2 of the middle one, and flipped toward the target origin.

Neighbor search is a 27-cell hash whose cell is 0.25 m. That cell is not the 5 cm downsample grid. A 27-cell neighborhood covers a query radius only when the cell is at least that radius, so the cell is the maximum of the normal radius (0.20 m), the initial correspondence gate (0.25 m), and the fixed inlier radius (0.10 m). The gate shrinks to `max(0.05 m, 3 × plane RMS)` and never grows past the cell. The hash refuses a radius larger than its cell. On a small cloud the same query matches a brute-force search.

Each step is a left perturbation on the parent frame. With `p'` the transformed source point, `q` the target point and `n` its normal, `J = [n | (p' × n) / L]`, residual `n · (p' − q)`, `L = 1 m`. The solve is weighted least squares for `(δt, L δω)`, then `R ← Exp(δω) R` and `t ← Exp(δω) t + δt`. Points are transformed as `((r0 x + r1 y) + r2 z) + t`, and a non-finite result is rejected. Stop when the step is under 0.1 mm and 0.01°, or at 25 steps.

Observability uses that same scaled, weighted Jacobian on the trimmed inliers. Let `λ_min` and `λ_max` be the smallest and largest eigenvalues of `JᵀJ`. Both are divided by the row count when they are reported; one half of the decision is the ratio `λ_min / λ_max`, which equals `(λ_min / N) / (λ_max / N)`. Comparing `λ_min / N` with the raw `λ_max` would make the answer depend on how many times each sample was copied, so that comparison is not the test. Scaling the rotational columns by `L` comes first, so the two eigenvalues share units. The ratio has to be at least `1e-4`.

The ratio alone is not the decision. A neighborhood PCA on a noisy plane tilts each normal, and that tilt leaks a little information into motions the surface does not constrain. With `λ0 ≤ λ1 ≤ λ2` the eigenvalues of the neighborhood covariance and `N` the neighbor count, the tilt variance toward the weaker in-plane axis is `σ² = λ0 / (N λ1)`. Over the weighted inliers the noise floor is `Σ w σ²`. On a single plane, on parallel planes, and on planes that share one axis, `λ_min` lands near that floor. A single plane with 1.5 cm of z noise has ratio about `6.6e-4`, above `1e-4`. The pose is observable only when the ratio clears `1e-4` and `λ_min` is at least twice the floor. Below either cut, the step is not applied. A floor, two parallel planes, planes whose normals are all perpendicular to one axis, and a two-wall corridor fail this test. Duplicating the rows multiplies both eigenvalues and the floor, and leaves the ratio, both per-row eigenvalues, and the comparison with the floor unchanged.

## Acceptance

Fixed-threshold metrics are computed on the seed and again on the estimate, with the same rules and no trimming: fraction of prepared source points with a target neighbor inside 0.10 m, and the RMS of `n · (p' − q)` on the neighbors inside that radius that have a normal. Snapshot B is scored with the frozen transform from A. It is not refit for that score.

`accepted` requires all of:

- the solve converged inside the step tolerance, rather than stopping because the iteration cap was hit
- observable, on the definition above
- at least 50 plane correspondences at the fixed 0.10 m radius
- translation moved no more than 0.05 m from the seed, and rotation no more than 3°
- fixed-threshold residual RMS ≤ 0.05 m and inlier fraction ≥ 0.15

Otherwise the JSON is still written, `accepted` is false, and the process exits 2. Exit 1 is a bad path, a bad cloud, a frame id other than `hesai_lidar` / `livox_frame`, or a bad `--init`. A result that is not accepted has `activation.eligible` false.

These gates mean the estimate stayed near the seed and landed on overlapping structure. They do not mean the mount was surveyed. The real snapshots have no ground-truth transform.

## What the recorded pair actually does

`Tests/Fixtures/room/snapshots/a` and `b` are CDR `sensor_msgs/PointCloud2` pairs. Header stamps differ by 169.492 ms on A and 44.590 ms on B. Counts: A is 115,200 JT128 points and 20,064 Mid-360 points; B is 115,200 and 19,968. Frame ids are `hesai_lidar` and `livox_frame`.

On A, JT128 z runs −0.393…2.561 m (median 1.427 m). Mid-360 raw z runs −1.139…1.175 m (median 0.205 m). After the nominal mount the Mid-360 median z is −0.315 m: the dome looks mostly downward, the JT128 mostly upward. For the 4,946 Mid-360 points at range ≥ 2 m, the distance between the raw livox coordinate and the nominal-mount coordinate has median 3.880 m and p90 5.855 m. That is the event-box gap below.

The tool, seeded with the nominal mount, on A:

| | seed | after refinement |
|---|---|---|
| inlier fraction at 0.10 m | 0.363 | 0.380 |
| plane RMS at 0.10 m | 0.0291 m | 0.0252 m |
| plane correspondences at 0.10 m | 1,587 | 1,643 |

Eight iterations, step tolerance met. The estimate moved 0.0350 m and 1.394° from the seed. Translation of the estimate is `(0.00852, 0.00253, −0.0762)` m against the seed `(0, 0, −0.110)`. The JT128 side hit the 20,000 point cap; the Mid-360 side kept 5,238 points after the range gate and the 5 cm voxels.

Observability on the final trimmed system (1,129 rows), characteristic length 1 m: `λ_min = 10.044`, `λ_max = 2488.2`, ratio `0.004037`. Dividing both by the row count gives `0.00890` and `2.204`, and the same ratio. The ratio clears `1e-4` (condition number about 248). The normal-estimation floor on those rows is `2.357`, so `λ_min` is `4.26` times the floor and clears the factor of two. A single plane at 1.5 cm of noise reaches only about `0.9` times its own floor, which is why the ratio cut is not sufficient there. An earlier scratch fit described the motion as observable without this eigenvalue; the number above is the one this solver produces. The ratio is well above a single plane and well below a densely overlapping room, which matches the two fields of view.

The seed's fixed-threshold RMS was already under 0.05 m, and the fraction was already above 0.15. Refinement changed the RMS by about 4 mm. Meeting the envelope is not evidence that the 3.5 cm shift is the true mount.

Held out on B, with A's transform and no further iteration: fraction 0.382, RMS 0.0246 m, 1,621 correspondences. The nominal seed on B, also without iteration: fraction 0.364, RMS 0.0284 m, 1,526 correspondences. An independent fit of B also passes the gates (25 iterations, ratio `0.00481`) and differs from A's estimate by 0.0133 m and 0.578°. That is repeatability across the two snapshots. It is not an accuracy against a surveyed transform.

## JSON and where it would live

The tool writes a candidate. Schema `labcommand.extrinsic.v1`, UTF-8, two-space indent, trailing newline. Fields include the 4×4 `T_parent_child` (`R` in the top-left, `t` in the last column, bottom row `0 0 0 1`), `translation_m`, `quaternion_xyzw` with `w ≥ 0`, the nominal mount, `T_init`, the solver settings (ranges, 5 cm voxel, 0.25 m search cell, `L`, trim, Huber, iteration and acceptance limits), fixed-threshold metrics before and after, the eigenvalues, the ratio, the normal-estimation floor and its margin, `accepted`, and `reject_reason`.

`room_level.status` is `deferred`. The file is `T_jt128←mid360` only.

Activation, later, in the room state directory from phase 3 §2.3:

| file | meaning |
|---|---|
| `~/room-memory/<room_id>/T_jt128_mid360.json` | the active mount, copied from a candidate with `accepted` true |
| `~/room-memory/<room_id>/T_room_jt128.json` | the deferred levelling `T_room←jt128` |

A candidate with `accepted` false is not copied onto the authoritative path and cannot become the active calibration. This tool never writes that directory. Exit 2 is a visible rejection, not a calibration.

Recalibrating the non-frame sensor keeps the room frame. The node invalidates the fused reference, rebuilds it (BUILDING), bumps `map_epoch`, and publishes a snapshot. Changing `T_room←frame_sensor`, or which sensor is the frame sensor, starts a new `room_id` generation. Both rules are already in phase 3 §2.3. Which sensor is the frame sensor stays Q10. This tool does not rebuild a reference and does not bump an epoch.

## Who applies it later

The room-memory node is the only stage that changes coordinates. Before integration and before ray casting it applies `T_room←sensor` to the points and to the ray origin (P14). The same transform is applied to every published geometric field, which is then already in the room frame:

- `ChangeEvent.centroid`
- `ChangeEvent.aabb`
- `ChangeEvent.from_centroid` (the tail of a moved arrow)
- the nested `mask` (brick indices in the room frame, not in the sensor frame)
- `Activity.centroid` and `Activity.aabb`

The Mac draws those fields directly. It does not apply `livoxToWorld` again to boxes, arrows, centroids, activity bounds, or masks. `sensors` and `largest_sensor` name the sensors that contributed. They do not say whether the geometry has already been transformed. After the node applies the extrinsic, a Mid-360 bit is still set on geometry that is already in the room frame.

There is no viewer-side correction in this change, temporary or otherwise. A later build that wants to draw events published under D30, from before the node applied the extrinsic, has to key that correction off the publication era: the `room_id` generation or the publisher generation at which the extrinsic became active. Keying it off the sensor bitmask would move those legacy boxes and would also move every later Mid-360 event whose geometry was already transformed.

## Event-box frame bug

Left in place. `memory.cpp`, `memory.py`, `LiveRoomView.swift`, and `RoomMapRenderer.swift` are unchanged.

Publisher, sensor frame. `RoomMemory::log_event` (`memory.cpp` 1045–1053) appends the CDR it was given. `RoomMemory::event` (1220–1248) fills it from that channel's region: `centroid`, `aabb_min` / `aabb_max`, `from_centroid` for a move, `sensors` as that channel's bit, and a nested mask whose occupied voxels are `candidate.keys`. The activity summary (1003–1006) copies the largest region's centroid and AABB the same way and sets `largest_sensor` to that region's bit. Protocol §3.7 says that bit is the frame those activity numbers are in. The map published beside the boxes is `channels_[map_channel_].reference` (`memory.cpp` 1089), the JT128 reference unless `map_sensor` says otherwise. The Python twin `_event` (`memory.py` 2005) copies the same fields.

Viewer, one coordinate space. `sensorWords` (`Sources/LabCommandCore/LiveRoomView.swift` 980–986) turns the bitmask into a name. Line 983 is the string `JT128`. It does not select a transform. `boxCorners` (988–993) and `marker` (995–1017) take `aabb`, and a moved arrow takes `fromCentroid` and `centroid`, with no call to `RigTransforms.toWorld`. `docs/LIVE-ROOM-PLAN.md` line 252 states the assumption the view implements: "`aabb` is meters in the room frame. No sensor transform." Protocol §3.2 says the same. D30 says the other sensor's events are not in that frame. The view follows §3.2. The node follows D30.

On snapshot A, a Mid-360 return at range ≥ 2 m is drawn about 3.88 m (p90 5.85 m) away from where the nominal mount places it in the JT128 map. JT128 events are already in the map frame, so those boxes land. The bug shows up when `sensors` contains Mid-360 and the map is the JT128. D48 records it. The JSON does not move the boxes.
