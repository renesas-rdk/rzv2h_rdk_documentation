#!/usr/bin/env python3

# Check which UM chapters have been modified compared to a given branch

import argparse
import glob
import re
import subprocess
import sys

from pathlib import Path


def make_test_name(rst_path: str) -> str:
    """
    Generate a stable test name from the RST file's relative path.

    Examples:
        source/chapter-01/intro.rst
        → um_test_c01_intro

        source/chapter-05/section-a/setup.rst
        → um_test_c05_section-a_setup

        source/chapter-12/sub/deep/page.rst
        → um_test_c12_sub_deep_page
    """
    p = Path(rst_path)
    parts = p.with_suffix("").parts  # remove .rst extension

    # Find the chapter part
    chapter_num = None
    chapter_idx = None
    for i, part in enumerate(parts):
        m = re.match(r"chapter-0?(\d+)", part)
        if m:
            chapter_num = int(m.group(1))
            chapter_idx = i
            break

    if chapter_num is None:
        return None

    # Everything after "chapter-XX/" becomes the suffix
    # e.g. ("section-a", "setup") → "section-a_setup"
    remaining = parts[chapter_idx + 1:]
    suffix = "_".join(remaining)

    return f"um_test_c{chapter_num:02}_{suffix}"


def check_diff_files(diff_list: list) -> None:
    """Identify which chapters contain changed files and print test names."""
    test_list = []

    rst_files = sorted(set(
        glob.glob("source/chapter-*/*.rst")
        + glob.glob("source/chapter-*/**/*.rst", recursive=True)
    ))

    for name in rst_files:
        if name not in diff_list:
            continue

        test_name = make_test_name(name)
        if test_name:
            test_list.append(test_name)

    print("|".join(test_list))


def get_diff_list(compare_to: str, path_str: str) -> list:
    """Get list of changed files by running git diff against the given branch."""
    result = subprocess.run(
        ["git", "diff", "--name-only", compare_to, "--", path_str],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        # Retry by adding "origin/" prefix
        result = subprocess.run(
            ["git", "diff", "--name-only", f"origin/{compare_to}", "--", path_str],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f'Error: "git diff --name-only {compare_to}" failed.')
            return []
    return result.stdout.splitlines()


def main() -> None:
    parser = argparse.ArgumentParser(description="Check updated UM files")
    parser.add_argument("compare_to", help="Branch name to compare.")
    parser.add_argument(
        "-p",
        "--path-str",
        default="source/",
        help="Path to compare (default: source/).",
    )

    args = parser.parse_args()

    if not Path(".git").exists():
        print(
            "check_diff.py: Please execute this script in the top of the repository directory."
        )
        sys.exit(1)

    diff_list = get_diff_list(args.compare_to, args.path_str)
    check_diff_files(diff_list)

    sys.exit(0)


if __name__ == "__main__":
    main()