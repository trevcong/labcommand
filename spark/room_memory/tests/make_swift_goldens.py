"""Regenerates ``fixtures/swift/*.json`` with the Swift reference CLI.

Run from anywhere on the Mac, with the repo's Swift toolchain:

    spark/.venv-local/bin/python spark/room_memory/tests/make_swift_goldens.py

It builds ``change-detect`` in release, writes each scene of ``goldens.SCENES`` as a
PointCloud2 ``.bin`` in a temporary directory, and stores the CLI's stdout. Phase 0 pairs
are run on the Phase 0 fixture files directly, when that directory exists.
"""

import pathlib
import subprocess
import sys
import tempfile

import goldens
import scenes

REPO = goldens.HERE.parents[2]


def run(binary, flags, before, after):
    result = subprocess.run([str(binary), *flags, str(before), str(after)], capture_output=True, check=True)
    return result.stdout


def main():
    subprocess.run(["swift", "build", "-c", "release", "--product", "change-detect"], cwd=REPO, check=True)
    binary = REPO / ".build" / "release" / "change-detect"
    goldens.SWIFT_DIR.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as scratch:
        scratch = pathlib.Path(scratch)
        for name, (build, flags) in goldens.SCENES.items():
            before, after = build()
            before_path = scratch / f"{name}-before.bin"
            after_path = scratch / f"{name}-after.bin"
            before_path.write_bytes(scenes.encode_pointcloud2(before))
            after_path.write_bytes(scenes.encode_pointcloud2(after))
            (goldens.SWIFT_DIR / f"{name}.json").write_bytes(run(binary, flags, before_path, after_path))
            print(f"wrote {name}.json")
    if not goldens.PHASE0_DIR.is_dir():
        print(f"skipped Phase 0 pairs: {goldens.PHASE0_DIR} is missing", file=sys.stderr)
        return
    for name, before, after in goldens.phase0_pairs():
        output = run(binary, goldens.EVIDENCE, goldens.PHASE0_DIR / before, goldens.PHASE0_DIR / after)
        (goldens.SWIFT_DIR / f"{name}.json").write_bytes(output)
        print(f"wrote {name}.json")


if __name__ == "__main__":
    main()
