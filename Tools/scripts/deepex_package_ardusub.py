#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
Archive a compiled ArduSub binary under the DeepEx naming rule

    ardusub-<commit8>-<target>

The binary is copied byte for byte into the archive directory, next to

    ardusub-<commit8>-<target>.sha256          (sha256sum -c compatible)
    ardusub-<commit8>-<target>.manifest.json

The binary is attributed to the Git commit checked out in --repo, so the
script refuses to run when the working tree has uncommitted changes, when the
target cannot be verified, or when the binary does not report that commit.
An existing archive entry is never overwritten: archiving the same bytes again
is a no-op, different bytes under the same name are an error.

Works with detached HEAD and shallow clones, and on Windows (set DEEPEX_GIT or
--git if git is not on PATH).

Example, after a successful "./waf configure --board navigator64 && ./waf sub":

    Tools/scripts/deepex_package_ardusub.py \\
        --binary build/navigator64/bin/ardusub \\
        --archive-dir /archive

Exit status: 0 on success (including "already archived"), 1 on any refusal.
"""

import argparse
import datetime
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys

SCHEMA_VERSION = 1
BINARY_NAME = "ardusub"
ARCHIVE_ENV = "DEEPEX_FIRMWARE_ARCHIVE"
GIT_ENV = "DEEPEX_GIT"

TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
CHUNK = 1 << 20


class PackagingError(Exception):
    """A refusal to archive; the message is shown to the user."""


def find_git(git=None):
    candidate = git or os.environ.get(GIT_ENV) or shutil.which("git")
    if not candidate:
        raise PackagingError("git not found: put it on PATH, set %s or pass --git" % GIT_ENV)
    return candidate


def run_git(git, repo, *args):
    cmd = [git, "-c", "safe.directory=%s" % repo, "-C", str(repo)] + list(args)
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    if proc.returncode != 0:
        raise PackagingError("git %s failed: %s" % (" ".join(args), proc.stderr.strip()))
    return proc.stdout


def repo_state(repo, git):
    """Return the identity of the checkout; raise if it is not a clean commit."""
    toplevel = pathlib.Path(run_git(git, repo, "rev-parse", "--show-toplevel").strip())
    commit = run_git(git, toplevel, "rev-parse", "--verify", "HEAD").strip().lower()
    if not COMMIT_RE.match(commit):
        raise PackagingError("unexpected commit id from git: %r" % commit)

    # Tracked changes, staged changes, modified submodules and untracked files
    # that are not ignored: any of them means the binary may not match the commit.
    status = run_git(git, toplevel, "status", "--porcelain=v1", "--untracked-files=normal")
    dirty = [line for line in status.splitlines() if line]
    if dirty:
        shown = "\n  ".join(dirty[:10])
        more = "" if len(dirty) <= 10 else "\n  ... and %d more" % (len(dirty) - 10)
        raise PackagingError(
            "refusing to archive: working tree has uncommitted changes, the binary cannot be "
            "attributed to commit %s:\n  %s%s" % (commit[:8], shown, more))

    branch = run_git(git, toplevel, "branch", "--show-current").strip()
    shallow = run_git(git, toplevel, "rev-parse", "--is-shallow-repository").strip() == "true"
    return {
        "toplevel": toplevel,
        "commit": commit,
        "commit8": commit[:8],
        "branch": branch or None,
        "detached_head": not branch,
        "shallow_clone": shallow,
    }


def infer_target(binary):
    """Return <target> for .../build/<target>/bin/ardusub, else None."""
    parts = binary.resolve().parts
    for i in range(len(parts) - 3, -1, -1):
        if parts[i] == "build" and parts[i + 2] == "bin":
            return parts[i + 1]
    return None


def validate_target(target):
    if not TARGET_RE.match(target):
        raise PackagingError("invalid target name %r" % target)
    return target


def read_waf_board(build_root, target):
    """BOARD recorded by 'waf configure' for this target, or None if not available."""
    cache = build_root / "c4che" / ("%s_cache.py" % target)
    if not cache.is_file():
        return None
    match = re.search(r"^BOARD\s*=\s*'([^']*)'", cache.read_text(errors="replace"), re.M)
    return match.group(1) if match else None


def read_header_git_version(target_dir):
    """GIT_VERSION from the build's ap_version.h, or None if not available."""
    header = target_dir / "ap_version.h"
    if not header.is_file():
        return None
    match = re.search(r'^#define\s+GIT_VERSION\s+"([^"]*)"', header.read_text(errors="replace"), re.M)
    return match.group(1) if match else None


def embedded_version(binary, commit8):
    """The 'ArduSub V... (<commit8>)' string baked into the binary, or None."""
    pattern = re.compile(rb"ArduSub V[0-9][^\x00\r\n]*\(" + re.escape(commit8.encode()) + rb"\)")
    tail = b""
    with open(binary, "rb") as f:
        while True:
            block = f.read(CHUNK)
            if not block:
                return None
            data = tail + block
            match = pattern.search(data)
            if match:
                return match.group(0).decode("ascii", "replace")
            tail = data[-512:]


def verify_target(binary, requested, state):
    """Settle the target name and return (target, list of checks that passed)."""
    inferred = infer_target(binary)
    if requested and inferred and requested != inferred:
        raise PackagingError("--target %s does not match the build directory %s of %s" % (requested, inferred, binary))
    target = requested or inferred
    if not target:
        raise PackagingError("cannot infer the target from %s: pass --target or use build/<target>/bin/ardusub" % binary)
    validate_target(target)

    checks = []
    if inferred:
        checks.append("path build/%s/bin/ardusub" % target)
        target_dir = binary.resolve().parent.parent
        build_root = target_dir.parent

        board = read_waf_board(build_root, target)
        if board is not None:
            if board != target:
                raise PackagingError("waf cache of build/%s says BOARD=%s" % (target, board))
            checks.append("waf cache BOARD=%s" % board)

        header = read_header_git_version(target_dir)
        if header is not None:
            if header != state["commit8"] and not state["commit"].startswith(header):
                raise PackagingError(
                    "build/%s/ap_version.h GIT_VERSION=%s but the checkout is %s: the build is stale or from another commit"
                    % (target, header, state["commit8"]))
            checks.append("ap_version.h GIT_VERSION=%s" % header)
    return target, checks


def sha256_of(path):
    digest = hashlib.sha256()
    size = 0
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(CHUNK), b""):
            digest.update(block)
            size += len(block)
    return digest.hexdigest(), size


def read_recorded_sha(sha_file):
    fields = sha_file.read_text().split()
    return fields[0].lower() if fields else ""


def write_new(path, data):
    """Create path with data; fail rather than replace an existing file."""
    try:
        with open(path, "xb") as f:
            f.write(data)
    except FileExistsError:
        raise PackagingError("%s appeared while archiving; not overwriting" % path)


def package(binary, repo, archive_dir, target=None, git=None):
    """Archive binary; return a dict describing the archive entry."""
    binary = pathlib.Path(binary)
    if binary.name != BINARY_NAME:
        raise PackagingError("%s is not an %s binary" % (binary, BINARY_NAME))
    if not binary.is_file():
        raise PackagingError("binary not found: %s" % binary)
    if binary.stat().st_size == 0:
        raise PackagingError("binary is empty: %s" % binary)

    git = find_git(git)
    state = repo_state(pathlib.Path(repo).resolve(), git)
    target, checks = verify_target(binary, target, state)

    version = embedded_version(binary, state["commit8"])
    if version is None:
        raise PackagingError(
            "%s does not report 'ArduSub V... (%s)': it was not built from this commit" % (binary, state["commit8"]))
    checks.append("embedded version string")

    name = "%s-%s-%s" % (BINARY_NAME, state["commit8"], target)
    archive_dir = pathlib.Path(archive_dir)
    archive_dir.mkdir(parents=True, exist_ok=True)
    dest = archive_dir / name
    sha_file = archive_dir / (name + ".sha256")
    manifest_file = archive_dir / (name + ".manifest.json")

    sha, size = sha256_of(binary)

    # Anything already archived under this name must carry the same bytes.
    for existing in (dest, sha_file, manifest_file):
        if not existing.exists():
            continue
        if existing == dest:
            recorded = sha256_of(dest)[0]
        elif existing == sha_file:
            recorded = read_recorded_sha(sha_file)
        else:
            try:
                recorded = json.loads(manifest_file.read_text()).get("sha256", "")
            except ValueError:
                recorded = ""
        if recorded != sha:
            raise PackagingError(
                "%s already exists with different content (archived %s, new %s); not overwriting"
                % (existing, recorded[:16] or "unreadable", sha[:16]))

    created = []
    if not dest.exists():
        try:
            with open(binary, "rb") as src, open(dest, "xb") as dst:
                shutil.copyfileobj(src, dst, CHUNK)
        except FileExistsError:
            raise PackagingError("%s appeared while archiving; not overwriting" % dest)
        shutil.copymode(binary, dest)
        created.append(dest)
        if sha256_of(dest)[0] != sha:
            raise PackagingError("copy of %s does not match the source; %s left for inspection" % (binary, dest))

    if not sha_file.exists():
        write_new(sha_file, ("%s  %s\n" % (sha, name)).encode())
        created.append(sha_file)

    manifest = {
        "schema": SCHEMA_VERSION,
        "name": name,
        "target": target,
        "commit": state["commit"],
        "commit8": state["commit8"],
        "branch": state["branch"],
        "detached_head": state["detached_head"],
        "shallow_clone": state["shallow_clone"],
        "working_tree_clean": True,
        "sha256": sha,
        "size_bytes": size,
        "embedded_version": version,
        "verified_by": checks,
        "source_binary": str(binary),
        "packaged_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "packager": pathlib.Path(__file__).name,
    }
    if not manifest_file.exists():
        write_new(manifest_file, (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode())
        created.append(manifest_file)

    return {
        "name": name,
        "path": dest,
        "sha256": sha,
        "size": size,
        "target": target,
        "commit": state["commit"],
        "created": created,
    }


def main():
    parser = argparse.ArgumentParser(description="Archive an ArduSub binary as ardusub-<commit8>-<target>")
    parser.add_argument("--binary", required=True, help="compiled ardusub, normally build/<target>/bin/ardusub")
    parser.add_argument("--repo", default=".", help="Git checkout the binary was built from (default: current directory)")
    parser.add_argument("--archive-dir", default=os.environ.get(ARCHIVE_ENV),
                        help="persistent archive directory (default: $%s)" % ARCHIVE_ENV)
    parser.add_argument("--target", help="board name; inferred from build/<target>/bin/ardusub when omitted")
    parser.add_argument("--git", help="git executable (default: $%s or PATH)" % GIT_ENV)
    args = parser.parse_args()

    if not args.archive_dir:
        parser.error("--archive-dir or $%s is required" % ARCHIVE_ENV)

    try:
        result = package(args.binary, args.repo, args.archive_dir, target=args.target, git=args.git)
    except PackagingError as e:
        print("deepex_package_ardusub: %s" % e, file=sys.stderr)
        return 1

    status = "archived" if result["created"] else "already archived (identical content)"
    print("%s: %s" % (status, result["path"]))
    print("commit  %s" % result["commit"])
    print("target  %s" % result["target"])
    print("sha256  %s" % result["sha256"])
    print("size    %d bytes" % result["size"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
