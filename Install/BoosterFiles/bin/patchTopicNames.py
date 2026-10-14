#!/usr/bin/env python3

# This is a utility script.
# DO NOT CALL IT DIRECTLY - unless you REALLY know what you are doing.
# It is called from enableRoboEireann.

import sys
import os
import shutil

script = os.path.basename(sys.argv[0])

USAGE = f"Usage: {script} patch|restore|check binary_path backup_path old_str new_str"


def error_exit(msg, code=1):
    print(f"{script}: ERROR: {msg}", file=sys.stderr)
    sys.exit(code)


def read_binary(path, label):
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError as e:
        error_exit(f"cannot read {label} {path}: {e}")


def report_string_presence(data, old, new, label, path):
    """Print presence/absence of old and new strings in data. Returns (has_old, has_new)."""
    has_old = old in data
    has_new = new in data
    print(f"{script}:   {label} ({path}):")
    print(f"{script}:     old string {old[:40]!r}: {'FOUND' if has_old else 'absent'}")
    print(f"{script}:     new string {new[:40]!r}: {'FOUND' if has_new else 'absent'}")
    return has_old, has_new


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

if len(sys.argv) != 6:
    print(USAGE, file=sys.stderr)
    sys.exit(1)

mode, binary_path, backup_path, old_str, new_str = sys.argv[1:]

if mode not in ("patch", "restore", "check"):
    error_exit(f"unknown mode '{mode}'. {USAGE}")

old = old_str.encode()
new = new_str.encode()

if len(new) > len(old):
    error_exit("replacement string must not be longer than original")

# ---------------------------------------------------------------------------
# CHECK mode — report state of in-place file and backup, exit non-zero if
# anything looks unexpected (backup missing or backup already patched).
# ---------------------------------------------------------------------------

if mode == "check":
    print(f"{script}: Checking state...")

    if not os.path.exists(binary_path):
        error_exit(f"binary not found: {binary_path}")

    binary_data = read_binary(binary_path, "binary")
    report_string_presence(binary_data, old, new, "binary", binary_path)

    if not os.path.exists(backup_path):
        print(f"{script}: WARNING: backup does not exist: {backup_path}")
        sys.exit(2)

    backup_data = read_binary(backup_path, "backup")
    backup_has_old, backup_has_new = report_string_presence(backup_data, old, new, "backup", backup_path)

    sizes_match = os.path.getsize(binary_path) == os.path.getsize(backup_path)
    print(f"{script}:   sizes match: {sizes_match}")

    if not backup_has_old:
        if backup_has_new:
            error_exit("backup contains the NEW string — backup looks already patched, not safe to restore", code=3)
        else:
            error_exit("backup contains neither old nor new string — backup may be corrupt", code=4)

    print(f"{script}: Check OK — backup looks valid")
    sys.exit(0)

# ---------------------------------------------------------------------------
# RESTORE mode — verify backup is good before touching the in-place binary.
# ---------------------------------------------------------------------------

if mode == "restore":
    if not os.path.exists(backup_path):
        error_exit(f"backup not found: {backup_path}")

    sizes_match = os.path.getsize(binary_path) == os.path.getsize(backup_path)
    if not sizes_match:
        error_exit(f"backup {backup_path} and binary {binary_path} sizes do not match - cannot restore", code=4)


    binary_data = read_binary(binary_path, "binary")
    if old in binary_data:
        print(f"binary {binary_path} already contains the ORIGINAL string {old_str}")
        sys.exit(0)


    backup_data = read_binary(backup_path, "backup")

    if old not in backup_data:
        if new in backup_data:
            error_exit(
                f"backup {backup_path} contains the NEW string, not the original one — "
                "refusing to restore a patched backup",
                code=3,
            )
        else:
            error_exit(
                f"backup {backup_path} contains neither original nor new string — "
                "backup may be corrupt or from a different binary version",
                code=4,
            )

    try:
        shutil.copy2(backup_path, binary_path)
        print(f"{script}: Restored {binary_path} from {backup_path}")
    except OSError as e:
        error_exit(f"failed to restore {binary_path} from {backup_path}: {e}")

    sys.exit(0)

# ---------------------------------------------------------------------------
# PATCH mode
# ---------------------------------------------------------------------------

if mode == "patch":
    if not os.path.exists(binary_path):
        error_exit(f"binary not found: {binary_path}")

    binary_data = read_binary(binary_path, "binary")

    if old not in binary_data:
        if new in binary_data:
            print(f"{script}: INFO: {binary_path} already patched (new string present), nothing to do")
            sys.exit(0)
        else:
            error_exit(
                f"neither old nor new string found in {binary_path} — "
                "wrong binary or wrong strings supplied"
            )

    # Decide whether to overwrite an existing backup.
    if os.path.exists(backup_path):
        backup_data = read_binary(backup_path, "backup")
        backup_has_old = old in backup_data
        sizes_match = os.path.getsize(binary_path) == os.path.getsize(backup_path)

        if backup_has_old and sizes_match:
            print(f"{script}: Backup already valid and sizes match, keeping existing backup: {backup_path}")
        else:
            if backup_has_old and not sizes_match:
                print(
                    f"{script}: WARNING: backup has old string but size differs from binary "
                    f"(binary vs backup: {os.path.getsize(binary_path)} vs {os.path.getsize(backup_path)} bytes) "
                    "— binary version may have changed, refreshing backup"
                )
            elif not backup_has_old:
                print(
                    f"{script}: WARNING: existing backup does not contain old string — "
                    "overwriting with fresh backup"
                )
            try:
                shutil.copy2(binary_path, backup_path)
                print(f"{script}: Backup refreshed: {backup_path}")
            except OSError as e:
                error_exit(f"failed to refresh backup {backup_path}: {e}")
    else:
        try:
            shutil.copy2(binary_path, backup_path)
            print(f"{script}: Backup created: {backup_path}")
        except OSError as e:
            error_exit(f"failed to create backup {backup_path}: {e}")

    # Apply the patch.
    patched = binary_data.replace(old, new.ljust(len(old), b"\x00"))
    try:
        with open(binary_path, "wb") as f:
            f.write(patched)
    except OSError as e:
        error_exit(f"failed to write patched binary {binary_path}: {e}")

    print(f"{script}: Patched {binary_path}: {old!r} --> {new!r}")
    sys.exit(0)

# Unreachable — mode was validated above — but makes the control flow explicit.
error_exit(f"unhandled mode '{mode}'")
