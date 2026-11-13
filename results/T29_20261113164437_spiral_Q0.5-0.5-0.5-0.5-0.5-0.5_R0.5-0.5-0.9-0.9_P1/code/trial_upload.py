"""Commit and push one completed trial, leaving other edits alone."""
import os
from pathlib import Path
import subprocess
import shutil


def upload_trial(folder):
    folder = Path(folder).resolve()
    git_directory = folder
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0")

    def git(*args, show=False):
        return subprocess.run(
            ["git", "-C", str(git_directory), *args], check=True,
            capture_output=not show, text=True, timeout=120, env=env,
        ).stdout or ""

    try:
        root = Path(git("rev-parse", "--show-toplevel").strip()).resolve()
        git_directory = root
        relative = folder.relative_to(root).as_posix()
        remote = git("remote", "get-url", "origin").strip()
        if not remote.rstrip("/").removesuffix(".git").endswith(
            ("github.com/Neel49/MTE544_lab", "github.com:Neel49/MTE544_lab")
        ):
            raise ValueError("origin must point to Neel49/MTE544_lab, not the course repo")
        branch = git("branch", "--show-current").strip()
        if branch != "LabThreeNirmit":
            raise ValueError("switch to LabThreeNirmit before uploading trials")

        git("add", "--", relative)
        if git("diff", "--cached", "--name-only", "--", relative):
            # --only excludes unrelated files that were already staged.
            git("commit", "--only", "-m", "Record trial " + folder.name, "--", relative)
        print("Pushing trial to GitHub...", flush=True)
        # Use the GitHub CLI login on this computer for HTTPS pushes.
        # These options apply only to this push; no Git settings are changed.
        auth = []
        if remote.startswith("https://") and shutil.which("gh"):
            logged_in = subprocess.run(
                ["gh", "auth", "status", "--hostname", "github.com"],
                capture_output=True, timeout=30, env=env,
            ).returncode == 0
            if logged_in:
                auth = ["-c", "credential.helper=", "-c",
                        "credential.helper=!gh auth git-credential"]
        git(*auth, "push", "origin", "HEAD:refs/heads/LabThreeNirmit", show=True)
        print("Uploaded trial: " + folder.name)
        return True
    except (subprocess.SubprocessError, OSError, ValueError) as error:
        detail = getattr(error, "stderr", None) or str(error)
        print("Trial saved locally, but upload failed: " + detail.strip())
        print("Resolve the Git/network issue, then retry: python3 trial_upload.py " + str(folder))
        return False


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("folder")
    args = parser.parse_args()
    raise SystemExit(0 if upload_trial(args.folder) else 1)
