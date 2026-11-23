#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
Tests for Tools/scripts/deepex_package_ardusub.py: archive naming, checksums,
collision handling and commit identification (clean, detached, shallow, dirty).
"""

import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCRIPT = os.path.join(ROOT, "Tools", "scripts", "deepex_package_ardusub.py")
sys.path.insert(0, os.path.join(ROOT, "Tools", "scripts"))

import deepex_package_ardusub as pkg  # noqa: E402

GIT = shutil.which("git")
TARGET = "navigator64"


def git(repo, *args):
    return subprocess.run(
        [GIT, "-c", "user.name=Test", "-c", "user.email=test@example.com", "-c", "commit.gpgsign=false",
         "-c", "safe.directory=*", "-C", str(repo)] + list(args),
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True).stdout.strip()


def fake_binary(commit8, payload=b""):
    return (b"\x7fELF\x02\x01\x01\x00" + b"\x00" * 64 + payload +
            b"\x00ArduSub V4.8.0-dev (" + commit8.encode() + b")\x00" + b"\x01\x02" * 100)


def write_build(repo, commit8, target=TARGET, payload=b"", board=None, header=None):
    """Lay out build/<target>/bin/ardusub the way Waf does; return the binary path."""
    build = pathlib.Path(repo) / "build"
    (build / target / "bin").mkdir(parents=True, exist_ok=True)
    (build / "c4che").mkdir(exist_ok=True)
    binary = build / target / "bin" / "ardusub"
    binary.write_bytes(fake_binary(commit8, payload))
    (build / "c4che" / (target + "_cache.py")).write_text("BOARD = '%s'\n" % (board or target))
    (build / target / "ap_version.h").write_text('#define GIT_VERSION "%s"\n' % (header or commit8))
    return binary


@unittest.skipIf(GIT is None, "git is not available")
class PackageArduSubTest(unittest.TestCase):

    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix="deepex_pkg_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.repo = self.tmp / "repo"
        self.repo.mkdir()
        git(self.repo, "init", "-q")
        (self.repo / ".gitignore").write_text("build/\n")
        (self.repo / "source.cpp").write_text("int a;\n")
        git(self.repo, "add", ".")
        git(self.repo, "commit", "-q", "-m", "first")
        (self.repo / "source.cpp").write_text("int b;\n")
        git(self.repo, "commit", "-q", "-am", "second")
        self.commit = git(self.repo, "rev-parse", "HEAD")
        self.c8 = self.commit[:8]
        self.archive = self.tmp / "archive"
        self.binary = write_build(self.repo, self.c8)

    def package(self, binary=None, repo=None, **kwargs):
        return pkg.package(binary or self.binary, repo or self.repo, self.archive, **kwargs)

    # naming and commit identification

    def test_name_uses_commit8_and_target(self):
        result = self.package()
        name = "ardusub-%s-%s" % (self.c8, TARGET)
        self.assertEqual(result["name"], name)
        self.assertEqual(len(self.c8), 8)
        self.assertEqual(sorted(p.name for p in self.archive.iterdir()),
                         [name, name + ".manifest.json", name + ".sha256"])

    def test_manifest_records_full_commit_and_checks(self):
        result = self.package()
        manifest = json.loads((self.archive / (result["name"] + ".manifest.json")).read_text())
        self.assertEqual(manifest["commit"], self.commit)
        self.assertEqual(manifest["commit8"], self.c8)
        self.assertEqual(manifest["target"], TARGET)
        self.assertEqual(manifest["sha256"], result["sha256"])
        self.assertEqual(manifest["size_bytes"], self.binary.stat().st_size)
        self.assertTrue(manifest["working_tree_clean"])
        self.assertFalse(manifest["shallow_clone"])
        self.assertIn("waf cache BOARD=%s" % TARGET, manifest["verified_by"])
        self.assertIn("embedded version string", manifest["verified_by"])

    def test_detached_head(self):
        git(self.repo, "checkout", "-q", "--detach")
        result = self.package()
        manifest = json.loads((self.archive / (result["name"] + ".manifest.json")).read_text())
        self.assertTrue(manifest["detached_head"])
        self.assertIsNone(manifest["branch"])
        self.assertEqual(manifest["commit"], self.commit)

    def test_branch_is_recorded(self):
        branch = git(self.repo, "branch", "--show-current")
        manifest = json.loads((self.archive / (self.package()["name"] + ".manifest.json")).read_text())
        self.assertEqual(manifest["branch"], branch)
        self.assertFalse(manifest["detached_head"])

    def test_shallow_clone(self):
        clone = self.tmp / "shallow"
        subprocess.run([GIT, "clone", "-q", "--depth", "1", self.repo.as_uri(), str(clone)],
                       check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        binary = write_build(clone, self.c8)
        result = self.package(binary=binary, repo=clone)
        manifest = json.loads((self.archive / (result["name"] + ".manifest.json")).read_text())
        self.assertTrue(manifest["shallow_clone"])
        self.assertEqual(manifest["commit"], self.commit)

    def test_subdirectory_of_checkout_is_accepted(self):
        sub = self.repo / "Tools"
        sub.mkdir()
        self.assertEqual(self.package(repo=sub)["commit"], self.commit)

    # refusing sources that are not a clean commit

    def test_modified_tracked_file_refused(self):
        (self.repo / "source.cpp").write_text("int changed;\n")
        with self.assertRaisesRegex(pkg.PackagingError, "uncommitted changes"):
            self.package()
        self.assertFalse(self.archive.exists() and any(self.archive.iterdir()))

    def test_staged_change_refused(self):
        (self.repo / "source.cpp").write_text("int staged;\n")
        git(self.repo, "add", "source.cpp")
        with self.assertRaisesRegex(pkg.PackagingError, "uncommitted changes"):
            self.package()

    def test_untracked_source_refused_but_ignored_build_dir_is_not(self):
        self.package()  # build/ is ignored, so the build itself does not make the tree dirty
        (self.repo / "new_file.cpp").write_text("int c;\n")
        with self.assertRaisesRegex(pkg.PackagingError, "uncommitted changes"):
            self.package()

    # target verification

    def test_target_inferred_from_path(self):
        self.assertEqual(self.package()["target"], TARGET)

    def test_explicit_target_must_match_build_directory(self):
        with self.assertRaisesRegex(pkg.PackagingError, "does not match"):
            self.package(target="navigator")

    def test_target_required_outside_build_layout(self):
        loose = self.tmp / "loose" / "ardusub"
        loose.parent.mkdir()
        loose.write_bytes(fake_binary(self.c8))
        with self.assertRaisesRegex(pkg.PackagingError, "cannot infer the target"):
            self.package(binary=loose)
        self.assertEqual(self.package(binary=loose, target="bluerov")["name"], "ardusub-%s-bluerov" % self.c8)

    def test_invalid_target_name_refused(self):
        loose = self.tmp / "loose" / "ardusub"
        loose.parent.mkdir()
        loose.write_bytes(fake_binary(self.c8))
        for bad in ("../x", "a/b", "-x", "a b", ""):
            with self.assertRaises(pkg.PackagingError):
                self.package(binary=loose, target=bad)

    def test_waf_cache_board_mismatch_refused(self):
        write_build(self.repo, self.c8, board="navigator")
        with self.assertRaisesRegex(pkg.PackagingError, "BOARD=navigator"):
            self.package()

    def test_stale_ap_version_refused(self):
        write_build(self.repo, self.c8, header="deadbeef")
        with self.assertRaisesRegex(pkg.PackagingError, "stale or from another commit"):
            self.package()

    def test_binary_from_other_commit_refused(self):
        write_build(self.repo, "deadbeef", header=self.c8)
        with self.assertRaisesRegex(pkg.PackagingError, "not built from this commit"):
            self.package()

    def test_only_ardusub_binary_accepted(self):
        other = self.binary.with_name("arducopter")
        other.write_bytes(self.binary.read_bytes())
        with self.assertRaisesRegex(pkg.PackagingError, "not an ardusub binary"):
            self.package(binary=other)

    # checksums

    def test_checksum_file_matches_sha256sum_format(self):
        result = self.package()
        expected = hashlib.sha256(self.binary.read_bytes()).hexdigest()
        self.assertEqual(result["sha256"], expected)
        sha_text = (self.archive / (result["name"] + ".sha256")).read_text()
        self.assertEqual(sha_text, "%s  %s\n" % (expected, result["name"]))

    def test_archived_binary_is_byte_identical_and_source_untouched(self):
        before = self.binary.read_bytes()
        mtime = self.binary.stat().st_mtime_ns
        result = self.package()
        self.assertEqual(result["path"].read_bytes(), before)
        self.assertEqual(self.binary.read_bytes(), before)
        self.assertEqual(self.binary.stat().st_mtime_ns, mtime)

    # collisions

    def test_same_content_again_is_a_noop(self):
        first = self.package()
        manifest_path = self.archive / (first["name"] + ".manifest.json")
        manifest_before = manifest_path.read_text()
        second = self.package()
        self.assertEqual(second["created"], [])
        self.assertEqual(manifest_path.read_text(), manifest_before)
        self.assertEqual(second["sha256"], first["sha256"])

    def test_different_content_is_never_overwritten(self):
        first = self.package()
        archived = first["path"].read_bytes()
        write_build(self.repo, self.c8, payload=b"different build")
        with self.assertRaisesRegex(pkg.PackagingError, "different content"):
            self.package()
        self.assertEqual(first["path"].read_bytes(), archived)
        recorded = (self.archive / (first["name"] + ".sha256")).read_text().split()[0]
        self.assertEqual(recorded, first["sha256"])

    def test_mismatching_checksum_file_blocks_archiving(self):
        name = "ardusub-%s-%s" % (self.c8, TARGET)
        self.archive.mkdir()
        (self.archive / (name + ".sha256")).write_text("%s  %s\n" % ("0" * 64, name))
        with self.assertRaisesRegex(pkg.PackagingError, "different content"):
            self.package()
        self.assertFalse((self.archive / name).exists())

    def test_missing_sidecars_are_restored_for_identical_binary(self):
        first = self.package()
        (self.archive / (first["name"] + ".sha256")).unlink()
        second = self.package()
        self.assertEqual([p.name for p in second["created"]], [first["name"] + ".sha256"])

    # command line

    def run_cli(self, *extra):
        return subprocess.run(
            [sys.executable, SCRIPT, "--binary", str(self.binary), "--repo", str(self.repo),
             "--archive-dir", str(self.archive), "--git", GIT] + list(extra),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    def test_cli_success_and_refusal_exit_codes(self):
        ok = self.run_cli()
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertIn("ardusub-%s-%s" % (self.c8, TARGET), ok.stdout)
        write_build(self.repo, self.c8, payload=b"other")
        refused = self.run_cli()
        self.assertEqual(refused.returncode, 1)
        self.assertIn("different content", refused.stderr)


if __name__ == "__main__":
    unittest.main()
