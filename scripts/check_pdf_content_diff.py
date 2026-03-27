#!/usr/bin/env python3

import argparse
import hashlib
import io
import sys

from pypdf import PdfReader, PdfWriter


def normalized_pdf_sha256(path: str) -> str:
    reader = PdfReader(path)
    writer = PdfWriter()

    for page in reader.pages:
        writer.add_page(page)

    # Drop document metadata so timestamp/producer-only changes do not trigger commits.
    writer.add_metadata({})
    if hasattr(writer, "_ID"):
        writer._ID = None  # best effort: remove trailer ID variability

    buffer = io.BytesIO()
    writer.write(buffer)
    return hashlib.sha256(buffer.getvalue()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare two PDFs by normalized content (metadata-insensitive)."
    )
    parser.add_argument("old_pdf", help="Path to previous PDF")
    parser.add_argument("new_pdf", help="Path to newly generated PDF")
    args = parser.parse_args()

    old_sha = normalized_pdf_sha256(args.old_pdf)
    new_sha = normalized_pdf_sha256(args.new_pdf)

    print(f"Normalized old SHA: {old_sha}")
    print(f"Normalized new SHA: {new_sha}")

    return 0 if old_sha == new_sha else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"check_pdf_content_diff.py: {exc}", file=sys.stderr)
        sys.exit(2)
