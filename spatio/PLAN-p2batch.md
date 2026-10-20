# P2 batch plan

Work only in this worktree, in order: #56, #51, #54, #52, #49, #64.

1. Make fixture regeneration compare computed floats with a small explicit
   tolerance while retaining exact structure, counts, text, and binary checks.
   Prove roundoff is accepted and meaningful changes are rejected.
2. Replace replay comparison's arccos rotation angles with stable atan2 forms.
   Test identical, tiny, antipodal, and half-turn rotations.
3. Make repo-root JT128 pytest collection use the existing harness for missing
   upstream inputs. Preserve vendor bytes and explicitly report unavailable PCAP
   coverage; do not add a fake decoder or skip runnable tests.
4. Fail manifest validation when a bag with a recorded hash is missing.
   Preserve the deliberate unhashed-input behavior and test CLI exit status.
5. Make unavailable rosbags coverage explicit in the local gate, and fail when
   required. Add regression coverage for available/missing dependency handling.
   No Spark runtime changes.
6. Reject sparse-turn competing unit interpretations regardless of which band
   center is closer. Add the supplied synthetic repro and retain evidence-backed
   unambiguous cases. Existing thresholds remain provisional.

For each fix, inspect/reproduce, make the smallest change, and run the affected
package with `python3 -m pytest -q` from the repo root. No Spark contact, apps,
containers, GitHub operations, or historical-recording tuning. No Swift changes
are planned. Record exact results and deliberate specification corrections in
`spatio/FINDINGS-p2batch.md`, attempt a local commit, and stop. If git metadata is
not writable, leave the changes uncommitted and report it.
