# F7a / F8a output-stage plan
1. Read existing clearance result JSON plus its matching zone-definition JSON.
2. Require an explicit walk ID; take all settings and evidence from inputs.
3. Add `python3 -m spatio.clearance report` without changing the existing CLI.
4. Join by unique zone ID and require matching frames, rules and geometry depth.
5. Use the required-volume centre as a labelled map reference, not an obstacle pose.
6. Preserve upstream statuses, reasons, measurements, provisions and time ranges.
7. Reject malformed inputs and PASS records lacking sufficient recorded evidence.
8. Support empty result/definition lists without claiming the walk passed.
9. Write `report.json` with schema `spatio.clearance_report.v1`.
10. Write compact `report.md` with walk, settings, four counts and all FAIL rows.
11. Include measured hit clearance, required minimum and voxel bound distinctly.
12. Include every NOT_MEASURED / INDETERMINATE zone in a numbered recheck list.
13. Sort zones lexically by ID; use stable JSON keys, UTF-8 and LF endings.
14. Add no clock, random ID, recording path, sensor setting or planner.
15. Test four statuses, fidelity, determinism, empty lists and recheck ordering.
16. Test translated/scaled synthetic inputs, invalid PASS and CLI failures.
17. Run the full clearance suite and document exact results and limits.
18. Keep all changes here, leave them uncommitted for the orchestrator, then stop.
