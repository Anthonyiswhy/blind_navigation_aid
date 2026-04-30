#!/usr/bin/env python3
"""Upload BlindNav run logs to a dedicated GitHub branch.

This script keeps field logs out of the active working tree. It clones the
configured remote into a temporary directory, writes the supplied log files
under field_logs/, commits them, and pushes a log-only branch.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def run(cmd, cwd=None, check=True):
    print("+ " + " ".join(cmd), flush=True)
    result = subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.stdout:
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n", flush=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"command failed with exit {result.returncode}: {' '.join(cmd)}")
    return result


def git_output(repo_root, *args):
    result = run(["git", "-C", repo_root, *args], check=True)
    return result.stdout.strip()


def resolve_remote(repo_root, override):
    if override:
        return override
    return git_output(repo_root, "config", "--get", "remote.origin.url")


def branch_exists(remote, branch):
    result = run(["git", "ls-remote", "--exit-code", "--heads", remote, branch], check=False)
    return result.returncode == 0


def checkout_upload_branch(remote, branch, workdir):
    if branch_exists(remote, branch):
        run(["git", "clone", "--depth", "1", "--branch", branch, remote, workdir])
        return

    run(["git", "clone", "--depth", "1", remote, workdir])
    run(["git", "checkout", "--orphan", branch], cwd=workdir)
    tracked = run(["git", "ls-files"], cwd=workdir).stdout.splitlines()
    if tracked:
        run(["git", "rm", "-r", "--quiet", "--cached", "--"] + tracked, cwd=workdir)
    for name in os.listdir(workdir):
        if name == ".git":
            continue
        path = os.path.join(workdir, name)
        if os.path.isdir(path):
            shutil.rmtree(path)
        else:
            os.unlink(path)


def copy_logs(log_paths, workdir, run_id):
    dest_dir = Path(workdir) / "field_logs" / run_id[:8] / run_id
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for log_path in log_paths:
        src = Path(log_path).expanduser()
        if not src.exists() or not src.is_file():
            print(f"skip missing log: {src}", flush=True)
            continue
        dest = dest_dir / src.name
        shutil.copy2(src, dest)
        copied.append(dest)
    return copied


def ensure_git_identity(workdir):
    name = run(["git", "config", "user.name"], cwd=workdir, check=False).stdout.strip()
    email = run(["git", "config", "user.email"], cwd=workdir, check=False).stdout.strip()
    if not name:
        run(["git", "config", "user.name", "BlindNav Log Uploader"], cwd=workdir)
    if not email:
        run(["git", "config", "user.email", "blindnav-logs@users.noreply.github.com"], cwd=workdir)


def upload_logs(repo_root, remote, branch, log_paths, run_id):
    with tempfile.TemporaryDirectory(prefix="blindnav-log-upload-") as tmp:
        workdir = os.path.join(tmp, "repo")
        checkout_upload_branch(remote, branch, workdir)
        copied = copy_logs(log_paths, workdir, run_id)
        if not copied:
            print("no logs copied; nothing to upload", flush=True)
            return 0

        ensure_git_identity(workdir)
        run(["git", "add", "field_logs"], cwd=workdir)
        status = run(["git", "status", "--porcelain"], cwd=workdir).stdout.strip()
        if not status:
            print("logs already uploaded; no commit needed", flush=True)
            return 0
        run(["git", "commit", "-m", f"Upload BlindNav logs {run_id}"], cwd=workdir)
        run(["git", "push", "origin", f"HEAD:{branch}"], cwd=workdir)
        print(f"uploaded {len(copied)} log file(s) to branch {branch}", flush=True)
        return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=os.getcwd())
    parser.add_argument("--remote", default="")
    parser.add_argument("--branch", default="blindnav-field-logs")
    parser.add_argument("--log", action="append", default=[])
    parser.add_argument("--run-id", default="")
    args = parser.parse_args()

    repo_root = os.path.abspath(os.path.expanduser(args.repo_root))
    remote = resolve_remote(repo_root, args.remote)
    run_id = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return upload_logs(repo_root, remote, args.branch, args.log, run_id)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
