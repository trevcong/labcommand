# F1 first synthetic step
1. Keep all changes in spatio/clearance; no hardware or recording access.
2. Define strict versioned JSON zones with IDs, provision text and required volumes.
3. Support axis-aligned boxes and bounded offsets from axis-aligned planes.
4. Define metre/map-frame geometry and explicit clearance direction for each zone.
5. Keep example rules in a fixture labelled EXAMPLE, NOT APPROVED.
6. Parse timestamped, map-frame scan origins and first-return endpoints strictly.
7. Build a small sparse NumPy-based voxel grid by exact ray/voxel traversal.
8. Mark only traversed cells free; endpoints occupied; missing rays stay unknown.
9. Preserve free and occupied observations independently, including time and conflicts.
10. Enumerate every positive-volume voxel intersection with each required volume.
11. FAIL on occupied required cells outside the provisional uncertainty band.
12. INDETERMINATE is the explicit near-limit exception; it never clears occupancy.
13. Otherwise return NOT_MEASURED for any unknown cell; PASS requires all cells free.
14. Report measured endpoint clearance and conservative voxel clearance separately.
15. Expose voxel size, grid origin and provisional metric uncertainty as CLI inputs.
16. Generate one synthetic panel/box, hidden-zone and narrow-gap scene.
17. Test parser rejection, ray geometry, coverage, conflicts and uncertainty boundaries.
18. Test a translated/scaled synthetic scene with different timestamps and origins.
19. Provide JSON and short Markdown reports with evidence counts and time range.
20. Run python3 -m pytest -q spatio/clearance and a CLI smoke test.
21. Record exact results, design limits, run instructions and approval needs.
22. Commit locally only if .git is writable; otherwise leave changes uncommitted.
