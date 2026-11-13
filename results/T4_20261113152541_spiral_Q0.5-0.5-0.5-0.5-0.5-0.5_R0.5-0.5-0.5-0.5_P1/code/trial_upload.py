"""Commit and push one completed trial, leaving other edits alone."""
import os
from pathlib import Path
import subprocess


def upload_trial(folder):
    folder = Path(folder).resolve()
    git_directory = folder
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0")

    def git(*args):
        return subprocess.run(
            ["git", "-C", str(git_directory), *args], check=True,
            capture_output=True, text=True, timeout=60, env=env,
        ).stdout.strip()

    try:
        root = Path(git("rev-parse", "--show-toplevel")).resolve()
        git_directory = root
        relative = folder.relative_to(root).as_posix()
        remote = git("remote", "get-url", "origin")
        if not remote.rstrip("/").removesuffix(".git").endswith(
            ("github.com/Neel49/MTE544_lab", "github.com:Neel49/MTE544_lab")
        ):
            raise ValueError("origin must point to Neel49/MTE544_lab, not the course repo")
        branch = git("branch", "--show-current")
        if branch != "LabThreeNirmit":
            raise ValueError("switch to LabThreeNirmit before uploading trials")

        git("add", "--", relative)
        if git("diff", "--cached", "--name-only", "--", relative):
            # --only excludes unrelated files that were already staged.
            git("commit", "--only", "-m", "Record trial " + folder.name, "--", relative)
        git("push", "origin", "HEAD:refs/heads/LabThreeNirmit")
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
