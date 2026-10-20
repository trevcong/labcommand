# Shared wall scoring plan
1. Read both milestone plans and review section 8; inspect both conversion paths.
2. Extract wall evidence, patch provenance, and wall verdict into one shared module.
3. Require at least two verified preselected patches; auto suggestions cannot pass.
4. Preserve the stricter whole-map doubled-wall failure, including the pillar case.
5. Keep whole-map and patch residual/separation gates and incomplete-evidence handling.
6. Feed explicit patch report, origin, and frozen spec to series adapt.
7. Exercise both paths on synthetic sharp, failed, pillar, missing, and auto-patch sessions.
8. List every changed existing assertion and its reason; do not weaken tests.
9. Run python3 -m pytest -q spatio .github/scripts and record exact results.
10. Write plans/SPATIO-m1-wallunify.md with usage, limits, and approval needs.
11. Leave changes uncommitted; no Spark, app, GitHub, or other-worktree operations.
