# Wall patch: doubled-wall check fails safe (g8safe)

Review P1 (sp-rv-m1final.txt, measure.py:236): the 6 cm partial-overlap fixture with its normal_hint rotated 4 deg gave doubled=false, p95 4.14 cm, and both gates passed.

## Plan (written before coding)
1. Only a would-be pass can become NOT_EVALUABLE. A patch that already fails a gate stays a fail.
2. In `reconcile_axes`, a split seen on only one axis may give a fail, never a pass. A one-axis gap at or under the 5 cm gate is refused.
3. A new pass guard that does not use the hint: if the residual band along the fitted normal is at least the 5 cm separation gate, the patch cannot pass. A band that thick can hold two surfaces at the gate distance.
4. No angle-based pass blocker. With the hint rotated −4°, the hint sits 0.03° from the fit and the patch still passed, so an angle check would miss it. A clean 10° tilt must stay MEASURED.
5. Regressions, run through `measure_map`: the 6 cm overlap with its hint rotated 1–6° about z and x; 64 clutter points in the valley; a clean single wall with an exact hint, both gates True.

### Changes from the plan
- **Step 3: the band is the narrowest interval holding 80%.** It started as p97.5 − p2.5. That broke the existing `test_outliers_do_not_dominate_fit`: its 5% one-sided outliers make that band 39 cm, while p95 is 4.24 cm and passes. The guard now uses the narrowest interval holding 80% (= 1 − MIN_PEAK_MASS) of the signed distances. Any surface the split rule could count (≥ 20% of points) has points inside it, and clutter under 20% falls outside it. The existing test was not changed.
- **Step 3b (added during the second session): a second pass guard, the stepped band.** Step 3 alone left a measured blind spot. With a −4° hint, two sheets 5.2–7 cm apart still passed when they were end to end, had a 20 cm gap, or overlapped by 10 cm. The pooled fit tilts and absorbs the offset, so the band stays under 5 cm.
  - That is the exact case the P1 names, so I did not ship it as a known limit.
  - The new check only blocks passes. It never produces a split verdict, and it never turns a fail into NOT_EVALUABLE.
  - It does not use the hint, and it adds no angle threshold.
  - **This goes beyond plan step 4.** It is not an angle check, but it is an extra check the plan did not list.

## What changed
- `doubled.py`
  - `reconcile_axes`: when one axis says doubled=true and the other says false, the result is a measured fail only if the gap is above the gate. Otherwise the patch is NOT_EVALUABLE.
  - New `core_band()` and `thick_band_reason()` (step 3).
  - New `stepped_band()` and `stepped_band_reason()` (step 3b):
    - The patch is split in two along an in-plane direction, at every 1% of the points (`SPLIT_STEP`), with at least 20% (`MIN_PEAK_MASS`) on each side.
    - The model `distance = a + b·u + c·[past the split]` is fitted by least squares.
    - The tilt `b·u` is removed, and the 80% band is measured again.
    - Split directions turn through the wall plane every 15° (`SPLIT_TURN_DEG`), so a join at any angle is covered.
    - On one flat or smooth surface, `b` stays near zero and the band does not change. Two offset sheets split near their join give back the tilt the offset caused, and the band then shows the offset.
  - RULE_TEXT updated.
- `measure.py`: a patch that would pass both gates becomes NOT_EVALUABLE, with the reason in the report, when `core_band_m` ≥ 0.05 m or `stepped_band_m` ≥ 0.05 m. New report fields:
  - `residuals.core_band_m`;
  - `residuals.stepped_band_m` (null when the patch already fails, because it is only computed for would-be passes);
  - `doubled_wall_rule.pass_needs_band_under_m` (0.05), `core_band_fraction` (0.8), `stepped_band_split_step` (0.01), `stepped_band_turn_deg` (15).
- **Constants.** The gate (SEPARATION_GATE_M) and the 20% floor (MIN_PEAK_MASS) are reused. The two new constants, 1% and 15°, only set how finely the check searches. They were not fitted to any data, synthetic or recorded. Nothing depends on a recording.

## How to run
- Tests: `python3 -m pytest -q spatio/wallpatch`.
- Measuring: `python3 -m spatio.wallpatch measure MAP --patches PATCHES.json --out DIR`, as before.
- NOT_EVALUABLE was already a patch status, so downstream readers need no change.
- Cost, measured: the stepped band takes 0.08 s on a 6,000-point patch, 0.75 s at 50,000 points and 3.4 s at 200,000.

## Tests
- `spatio/wallpatch`: **32 passed** (24 existing + 8 new).
- `python3 -m pytest -q spatio .github/scripts`: **838 passed, 1 skipped** (89.9 s)
- New tests:
  - `test_rotated_hint_on_6cm_overlap_never_passes`: hint rotated ±1…6° about z and about x, n_cropped 1600 each time. No angle passes. The five angles that passed before are now NOT_EVALUABLE.
  - `test_one_axis_split_under_the_gate_is_not_a_pass`
  - `test_64_clutter_points_in_the_valley_do_not_pass`: exact hint and −4° hint.
  - `test_clean_single_wall_with_exact_hint_still_passes`: σ 5 mm. MEASURED, doubled=false, both gates True, both bands under 5 cm.
  - `test_thick_band_blocks_only_a_would_be_pass`: a uniform 7 cm slab becomes NOT_EVALUABLE. The 6 cm exact-hint fixture stays a measured fail with `stepped_band_m` null.
  - `test_offset_sheets_tilting_the_fit_are_not_a_pass` (end to end through `measure_map`): 7 offset-sheet cases with a −4° hint, all NOT_EVALUABLE with the stepped-band reason. They are end to end at 5.2 / 5.5 / 6 cm, a 20 cm gap at 5.2 / 6 / 7 cm, and a 10 cm overlap at 5.2 cm.
  - `test_offset_sheet_matrix_never_passes`: 1,260 cases, none passing. It varies:
    - layout: end to end, 20 cm gap, 10 cm or 20 cm overlap;
    - gap: 5.2, 6 or 7 cm;
    - join position: −0.3, −0.15, 0 or +0.2 m on a 1 m patch;
    - join direction: 0°, 30°, 45° or 90°;
    - hint: exact, or ±2–4° about z and x.
    
    Layouts where one sheet holds under 20% of the points are skipped, because that is the split rule's own floor.
  - `test_bowed_single_wall_still_passes`: 1 cm of bow over 1 m, σ 5 mm, exact hint. It still passes.
- **Old code fails the new tests.**
  - First session: with the old doubled.py and measure.py restored, the first 5 new tests failed (5 failed, 24 passed). The clean-wall test failed there only because the `core_band_m` key was missing.
  - This session: with only `stepped_band_reason` disabled, `test_offset_sheets_tilting_the_fit_are_not_a_pass` and `test_offset_sheet_matrix_never_passes` fail, and the matrix has 154 of 1,260 cases passing. The bowed-wall and clean-wall tests still pass.
- Swift: not run, because no Swift changed.

## Measured results (synthetic)
The 6 cm review fixture is 40×20 noiseless grids per sheet, at y = ±3 cm, overlapping over ±0.2 m.

| Case | Old | New |
|---|---|---|
| hint −6°, −4° about z; ±4° about x | doubled=false, PASS | NOT_EVALUABLE (80% band 6.77 cm) |
| hint −2° about z | doubled=true, gap 4.95 cm, PASS | NOT_EVALUABLE (one-axis gap under gate) |
| other angles, ±1…6° | measured fail | measured fail (unchanged) |
| + 64 clutter points at y=0, hint −4° | doubled=false, PASS | NOT_EVALUABLE |
| + 64 clutter points, exact hint | fail, gap 5.78 cm | fail (unchanged) |
| offset sheets, −4° hint (the first session's blind spot) | PASS | NOT_EVALUABLE (stepped band) |

Larger sweep, run with a scratch script outside the repo:
- **Noiseless and σ 5 mm sheets.** 8,232 cases in all: 2 noise levels, 7 join directions 0–90° in 15° steps, 7 join positions from 20% to 80% of the patch width, 4 layouts, gaps of 5.2 / 6 / 7 cm, and 7 hints.
- **No pass with both sheets ≥ 20% of the points, except one noisy case.** The only passes left with both sheets at ≥ 20% are one noisy configuration: σ 5 mm, a 5.2 cm gap, a 10 cm overlap, the join at +0.1 m, and the hint at −4° about z or about x. It passed for join directions 0°, 15° and 90°. Measured on that case: 80% band 4.89 cm, stepped band 4.96 cm, both just under the 5 cm gate.
- **Noisier sheets, σ 1 cm and 1.5 cm.** 4,116 cases each: gaps of 5.5 / 6 / 7 cm, the same layouts, positions, directions and hints. No case passes with both sheets at ≥ 20%. At σ 1 cm, 15 cases pass, all with one sheet under 20%; at σ 1.5 cm, none pass at all. Above σ ≈ 1.95 cm, the 80% band alone already refuses every would-be pass.
- **The other passes all have a sheet under 20%.** That is below the split rule's own floor, so the exact-hint rule cannot see that sheet either.

Clean walls (6,000 points, exact hint unless stated). All pass; values are the 80% band and the stepped band:

| Wall | 80% band | Stepped band |
|---|---|---|
| σ 5 mm | 1.25 cm | 1.26 cm |
| σ 1 cm | 2.51 cm | 2.52 cm |
| 10° tilt with the hint 10° off, σ 4 mm | 1.00 cm | 1.00 cm |
| 5% far outliers | 2.23 cm | 2.95 cm |
| 1 cm bow over 1 m, σ 5 mm | 1.47 cm | 1.61 cm |
| 2 cm bow over 1 m, σ 5 mm | 2.00 cm | 2.31 cm |

## Known limits
- **Near the gate under noise.** A 5.2 cm offset with 5 mm noise can still pass in one layout: its stepped band measured 4.96 cm. Offsets of 5.5 cm and more did not pass at σ 1 or 1.5 cm in the sweep above. 5.2 cm was not swept at those noise levels.
- **A sheet under 20% of the patch is not seen**, with or without a hint. This floor is the same as the split rule's (MIN_PEAK_MASS).
- **Stricter in effect.** A single Gaussian wall passes only if its 80% band is under 5 cm, which means σ ≲ 1.95 cm (p95 ≲ 3.8 cm). Walls with p95 between about 3.8 and 5 cm used to pass and are now NOT_EVALUABLE. Measured on a synthetic σ = 2 cm wall: 80% band 5.10 cm, so NOT_EVALUABLE. The stepped band adds little on top of this for smooth walls (see the table above). A wall with a real ≥ 5 cm feature over at least 20% of the patch, such as a shelf or deep trim, will now be NOT_EVALUABLE instead of a pass. That is intended, but it means patch boxes must avoid such features.
- **No angle check.** A clean wall whose hint is 10° off stays MEASURED, because an existing test requires that.
- **Only synthetic sheets were tested.** No real map was measured in this task.

## Needs Trevor's approval
1. The effective tightening, from "p95 ≤ 5 cm" to "p95 ≤ 5 cm and both 80% bands < 5 cm" for a pass.
2. The stepped-band guard itself. It goes beyond plan step 4 and adds two search constants: 1% split step and 15° direction step.
3. Whether the leftover near-gate case (5.2 cm offset under 5 mm noise) is acceptable for walk day, or whether a pass should need a margin below the gate. I did not add a margin, because that would be a new tuned number.
