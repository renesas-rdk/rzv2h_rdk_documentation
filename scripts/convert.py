import argparse
import os
import re
import fnmatch
import subprocess

from pypdf import PdfWriter

# ============================================================
# Empty Title Removal
# ============================================================

def remove_empty_titles(content):
    """
    Remove titles (== ... ) that have no content below them.
    Keep a title only if it is followed by actual content (not another title or EOF).
    """
    lines = content.split('\n')
    result = []
    i = 0

    while i < len(lines):
        line = lines[i]

        # Check if the line is a title (starts with =, ==, ===, etc.)
        if re.match(r'^={1,6}\s+\S', line):
            j = i + 1

            # Skip blank lines after the title
            while j < len(lines) and lines[j].strip() == '':
                j += 1

            # Check whether the next non-blank line is content or another title
            if j < len(lines) and not re.match(r'^={1,6}\s+\S', lines[j]):
                # There is content — keep the title
                result.append(line)
            else:
                # No content (EOF or another title immediately after) — skip
                print(f"  REMOVED empty title: {line.strip()}")
                # Also skip the blank lines that followed the title
                i = j
                continue
        else:
            result.append(line)

        i += 1

    return '\n'.join(result)


# ============================================================
# Skip List Configuration
# ============================================================

def load_skip_list(skip_file="skip_list.txt"):
    """
    Load the list of files/folders to skip from a config file.

    Supported formats:
      - Comments (#)
      - Folder names (e.g., chapter-03)
      - Explicit paths (e.g., source/chapter-01/debug.rst)
      - Wildcard patterns (e.g., **/draft_*)
    """
    skip_patterns = []

    if not os.path.exists(skip_file):
        print(f"INFO: No skip list file found at '{skip_file}', nothing will be skipped.")
        return skip_patterns

    with open(skip_file, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            skip_patterns.append(line)

    print(f"Loaded {len(skip_patterns)} skip patterns from '{skip_file}':")
    for p in skip_patterns:
        print(f"  - {p}")

    return skip_patterns


def should_skip(file_path, skip_patterns):
    """
    Check whether *file_path* matches any of the skip patterns.

    Matching strategies (tried in order):
      1. Exact path match
      2. Folder-name match (any path component equals the pattern)
      3. Basename match
      4. Wildcard/glob match on the full path
      5. Wildcard/glob match on the basename only
    """
    normalized_path = os.path.normpath(file_path)
    path_parts = normalized_path.split(os.sep)

    for pattern in skip_patterns:
        pattern = pattern.strip()

        # 1. Exact path match
        if os.path.normpath(pattern) == normalized_path:
            return True

        # 2. Folder-name match
        #    e.g. pattern "chapter-03" matches "source/chapter-03/intro.rst"
        if pattern in path_parts:
            return True

        # 3. Basename match
        #    e.g. pattern "debug.rst" matches "source/chapter-01/debug.rst"
        if os.path.basename(normalized_path) == pattern:
            return True

        # 4. Wildcard match on full path
        #    e.g. pattern "**/draft_*" matches "source/chapter-01/draft_intro.rst"
        if fnmatch.fnmatch(normalized_path, pattern):
            return True

        # 5. Wildcard match on filename only
        if fnmatch.fnmatch(os.path.basename(normalized_path), pattern):
            return True

    return False


# ============================================================
# Toctree Parsing (with skip support)
# ============================================================

def parse_toctree(rst_file_path, skip_patterns=None):
    """
    Parse an RST file, find all toctree directives, and return an ordered
    list of resolved file paths.

    Recursively descends into subfolders when a child file also contains
    toctree directives.  Files/folders matching *skip_patterns* are excluded.
    """
    if skip_patterns is None:
        skip_patterns = []

    if not os.path.exists(rst_file_path):
        print(f"WARNING: File not found: {rst_file_path}")
        return []

    base_dir = os.path.dirname(rst_file_path)

    with open(rst_file_path, "r", encoding="utf-8") as f:
        content = f.read()

    toctree_pattern = re.compile(
        r'\.\.\s+toctree::\s*\n'
        r'((?:[ \t]+\S.*\n)*)'
        r'((?:[ \t]*\n)*)'
        r'((?:[ \t]+\S.*\n?)*)',
        re.MULTILINE
    )

    ordered_files = []

    for match in toctree_pattern.finditer(content):
        entries_block = match.group(3)

        for line in entries_block.strip().splitlines():
            entry = line.strip()
            if not entry or entry.startswith(":") or entry.startswith(".."):
                continue

            # Handle entries with an explicit title: "Custom Title <path/to/file>"
            title_path_match = re.match(r'.*<(.+)>', entry)
            if title_path_match:
                entry = title_path_match.group(1).strip()

            # Resolve the file path
            entry_with_ext = entry if entry.endswith(".rst") else entry + ".rst"
            full_path = os.path.normpath(os.path.join(base_dir, entry_with_ext))

            # --- Skip checks ---
            if should_skip(full_path, skip_patterns):
                print(f"  SKIPPED: {full_path}")
                continue

            if should_skip(entry, skip_patterns):
                print(f"  SKIPPED (by entry name): {entry}")
                continue
            # -------------------

            if os.path.exists(full_path):
                ordered_files.append(full_path)
                ordered_files.extend(parse_toctree(full_path, skip_patterns))
            else:
                # Try treating the entry as a folder with an index.rst inside
                folder_index = os.path.normpath(
                    os.path.join(base_dir, entry, "index.rst")
                )

                if should_skip(folder_index, skip_patterns):
                    print(f"  SKIPPED (folder): {folder_index}")
                    continue

                if os.path.exists(folder_index):
                    ordered_files.append(folder_index)
                    ordered_files.extend(parse_toctree(folder_index, skip_patterns))
                else:
                    print(f"WARNING: Cannot resolve toctree entry: {entry}")
                    print(f"  Tried: {full_path}")
                    print(f"  Tried: {folder_index}")

    return ordered_files


# ============================================================
# RST Helper Utilities
# ============================================================

def remove_toctree_directives(content):
    """Strip all toctree directives from RST content."""
    pattern = re.compile(
        r'\.\.\s+toctree::.*?(?=\n\S|\n\n\S|\Z)',
        re.DOTALL
    )
    return pattern.sub('', content)


def get_chapter_from_path(file_path):
    """Extract the chapter folder name (e.g. 'chapter-01') from a file path."""
    parts = os.path.normpath(file_path).split(os.sep)
    for part in parts:
        if part.startswith("chapter-"):
            return part
    return None


# ============================================================
# Dump / Combine RST Files (with skip support)
# ============================================================

def dump_rst_files(base_folder, output_file="combined.rst", skip_patterns=None):
    """
    Combine RST files in toctree order (recursing into subfolders) into a
    single output file.  Files/folders matching *skip_patterns* are excluded.
    """
    if skip_patterns is None:
        skip_patterns = []

    root_index = os.path.join(base_folder, "index.rst")

    if not os.path.exists(root_index):
        print(f"ERROR: Root index not found: {root_index}")
        return

    ordered_files = parse_toctree(root_index, skip_patterns)

    print("=" * 60)
    print("Resolved toctree order (after skip filtering):")
    print("=" * 60)
    for i, f in enumerate(ordered_files, 1):
        print(f"  {i:3d}. {f}")
    print("=" * 60)

    if skip_patterns:
        print(f"\nSkip patterns applied: {skip_patterns}\n")

    if os.path.exists(output_file):
        os.remove(output_file)

    current_chapter = None

    with open(output_file, "a", encoding="utf-8") as outfile:
        # 1. Write the root index.rst first
        with open(root_index, "r", encoding="utf-8") as f:
            content = f.read()
            content = remove_toctree_directives(content)
            outfile.write(content + "\n\n")

        # 2. Write each file in toctree order
        for rst_path in ordered_files:
            chapter = get_chapter_from_path(rst_path)
            if chapter and chapter != current_chapter:
                if current_chapter is not None:
                    outfile.write("\n<<<\n\n")
                current_chapter = chapter

            with open(rst_path, "r", encoding="utf-8") as f:
                content = f.read()
                content = remove_toctree_directives(content)
                outfile.write(content + "\n\n")

    print(f"RST files have been combined into {output_file}")
    print(f"Total files: {len(ordered_files) + 1} (including root index)")


# ============================================================
# RST to AsciiDoc Conversion
# ============================================================

def rst_to_adoc(rst_file):
    """
    Convert a (combined) RST file to AsciiDoc via Pandoc, then apply
    post-processing fixes (image paths, admonition titles, attributes, etc.).
    """
    with open(rst_file, "r", encoding="utf-8") as file:
        rst_content = file.read()

    # Pre-process cross-references so Pandoc can handle them
    rst_content = re.sub(r':ref:`([^<]+)<([^>]+)>`', r'<<\2,\1>>', rst_content)
    rst_content = re.sub(r':ref:`([^<`]+)`', r'<<\1>>', rst_content)
    rst_content = re.sub(
        r':ref:`([^<]+?)\s*<([^>]+)>\s*`',
        r'<<\2,\1>>',
        rst_content,
    )

    # Normalise non-standard admonition types to the ones Pandoc understands
    rst_content = re.sub(r'^[ \t]*\.\.\s+attention::', '.. warning::', rst_content, flags=re.MULTILINE)
    rst_content = re.sub(r'^[ \t]*\.\.\s+danger::',    '.. warning::', rst_content, flags=re.MULTILINE)
    rst_content = re.sub(r'^[ \t]*\.\.\s+error::',     '.. warning::', rst_content, flags=re.MULTILINE)
    rst_content = re.sub(r'^[ \t]*\.\.\s+hint::',      '.. tip::',     rst_content, flags=re.MULTILINE)
    rst_content = re.sub(r'^[ \t]*\.\.\s+seealso::',   '.. note::',    rst_content, flags=re.MULTILINE)

    # Write a temporary pre-processed RST file for Pandoc
    temp_rst = rst_file.replace(".rst", "_processed.rst")
    with open(temp_rst, "w", encoding="utf-8") as file:
        file.write(rst_content)

    adoc_file = rst_file.replace(".rst", ".adoc")
    subprocess.run([
        "pandoc", "-f", "rst", "-t", "asciidoc",
        "--wrap=none", temp_rst, "-o", adoc_file,
    ])

    with open(adoc_file, "r", encoding="utf-8") as file:
        content = file.read()

    # Prepend AsciiDoc document header / attributes
    header = """\
:toc: macro
:sectnums:
:toclevels: 3
:sectnumlevels: 5
:toc-title: Contents
:source-highlighter: highlight.js
:highlightjs-theme: github
:experimental:
:pp: {plus}{plus}

toc::[]

<<<
"""
    content = header + "\n" + content

    # Remove section titles that have no content beneath them
    content = remove_empty_titles(content)

    # Fix image paths: ../../images/ → source/images/
    content = re.sub(
        r'image::\.\./\.\./images/',
        'image::source/images/',
        content,
    )
    # Fix image paths: ../images/ → source/images/
    content = re.sub(
        r'image::\.\./images/',
        'image::source/images/',
        content,
    )

    # Remove auto-generated admonition caption lines (e.g. ".Tip", ".Warning")
    content = re.sub(
        r'(\[(?:TIP|NOTE|WARNING|IMPORTANT|CAUTION)\])\n'
        r'\.(?:Tip|Note|Warning|Important|Caution|Attention|Danger|Error|Hint|See also)\n'
        r'(====)',
        r'\1\n\2',
        content,
    )

    # Convert {:.nonum .discrete} style attributes to AsciiDoc [discrete, nonum]
    attr_pattern = re.compile(
        r"^(=+)(\s+)(.*?)(\s*){:\s*((?:\.(?:nonum|discrete)\s*)+)}$",
        flags=re.MULTILINE,
    )

    def replace_attr(match):
        level = match.group(1)
        spaces = match.group(2)
        title = match.group(3).strip()
        attrs = match.group(5).strip().split()
        flags = [a.strip(".") for a in attrs if a in {".nonum", ".discrete"}]
        return f"[{', '.join(flags)}]\n{level}{spaces}{title}"

    content = attr_pattern.sub(replace_attr, content)

    with open(adoc_file, "w", encoding="utf-8") as file:
        file.write(content)


# ============================================================
# High-level Pipeline
# ============================================================

def convert_rst(args):
    skip_patterns = load_skip_list("scripts/skip_list.txt")

    if hasattr(args, 'skip') and args.skip:
        skip_patterns.extend(args.skip)
        print(f"Additional skip patterns from CLI: {args.skip}")

    combined_rst_file = "combined_manual.rst"
    dump_rst_files("source", combined_rst_file, skip_patterns)
    rst_to_adoc(combined_rst_file)

    # Build environment with overrides
    env = os.environ.copy()
    if args.release_version:
        env["PACKAGE_VER"] = args.release_version
    if args.rev_date:
        env["REV_DATE"] = args.rev_date

    subprocess.run(
        [
            "bash",
            "scripts/convert_pdf.sh",
            combined_rst_file.replace(".rst", ".adoc"),
            combined_rst_file.replace(".rst", ".pdf"),
        ],
        env=env,
    )


def merge_pdfs(output_path):
    """Merge cover, intro, and main-body PDFs into a single output file."""
    merger = PdfWriter()

    pdf_list = [
        "docs/configs/Cover.pdf",
        "docs/configs/Intro.pdf",
        "combined_manual.pdf",
    ]

    for pdf in pdf_list:
        merger.append(pdf)

    os.makedirs("output", exist_ok=True)
    merger.write(os.path.join("output", output_path))
    merger.close()

    print("User Manual PDF merged successfully!")


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert RST files to PDF and run pdflatex."
    )
    parser.add_argument(
        "--output_filename",
        help="The filename for the PDF output, e.g., report.pdf",
    )
    parser.add_argument(
        "--release_version",
        type=str,
        default=None,
        help="Package version string, e.g., 2.0 (overrides PACKAGE_VER in shell)",
    )
    parser.add_argument(
        "--rev_date",
        type=str,
        default=None,
        help="Revision date string, e.g., Jun.15.25 (overrides REV_DATE in shell)",
    )
    parser.add_argument(
        "--skip",
        nargs="*",
        default=[],
        help="List of files/folders to skip. E.g., --skip chapter-03 debug.rst",
    )

    args = parser.parse_args()
    convert_rst(args)
    merge_pdfs(args.output_filename)