import re
import shutil
from json import dumps
from pathlib import Path

from sphinx_polyversion import DefaultDriver, apply_overrides
from sphinx_polyversion.environment import Environment
from sphinx_polyversion.git import Git, file_predicate
from sphinx_polyversion.sphinx import SphinxBuilder


ROOT = Path(__file__).parent.resolve()
SOURCE_DIR = ROOT / "source"
OUTPUT_DIR = ROOT / "build" / "html"

# Build only release tags and the mainline branches.
TAG_RE = re.compile(r"^v\d+\.\d+\.\d+$")
BRANCH_RE = re.compile(r"^$")

data = Git(
    branch_regex=BRANCH_RE,
    tag_regex=TAG_RE,
    buffer_size=1 * 10**9,
    predicate=file_predicate(["source", "requirements.txt"]),
)

builder = SphinxBuilder(
    source="source",
    args=["-W"],
)

driver = DefaultDriver(
    root=ROOT,
    output_dir=OUTPUT_DIR,
    vcs=data,
    builder=builder,
    env=Environment.factory(),
)

apply_overrides(globals())
driver.output_dir = Path(OUTPUT_DIR)
driver.run()

STYLE_OVERRIDE = """
/* Polyversion runtime override */
.version-switcher__label { display: none !important; }
.version-switcher__select {
  appearance: none;
  -webkit-appearance: none;
  -moz-appearance: none;
  background-color: #2980b9;
  border: none !important;
  border-radius: 4px;
  background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 10 6'%3E%3Cpath fill='hsla(0%2C0%25%2C100%25%2C.65)' d='M1 1l4 4 4-4'/%3E%3C/svg%3E");
  background-position: right 0.45rem center;
  background-repeat: no-repeat;
  background-size: 0.62rem 0.4rem;
  box-shadow: none !important;
  color: hsla(0,0%,100%,.3);
  cursor: pointer;
  font-size: 0.9rem;
  line-height: 1.2;
  padding: 0.22rem 1.2rem 0.22rem 0.4rem;
  text-align: center;
  transition: background-color 0.2s ease;
  width: 40%;
  min-width: 7.5rem;
}
.version-switcher__select:hover {
  background-color: #1f6ea3;
}
.version-switcher__select:focus {
  background-color: #1f6ea3;
  border: none !important;
  box-shadow: none !important;
  outline: none !important;
}
.version-switcher__select option {
  background-color: #ffffff;
  color: #111111;
  text-align: center;
}
""".strip()


def tag_key(tag_name: str) -> tuple[int, int, int]:
    major, minor, patch = tag_name.lstrip("v").split(".")
    return int(major), int(minor), int(patch)


tag_dirs = sorted(
    [p for p in OUTPUT_DIR.iterdir() if p.is_dir() and TAG_RE.match(p.name)],
    key=lambda p: tag_key(p.name),
)

if tag_dirs:
    latest_src = tag_dirs[-1]
    history_tag_dirs = tag_dirs[:-1][-2:]

    # Build `latest` from highest tag.
    latest_dst = OUTPUT_DIR / "latest"
    if latest_dst.exists():
        shutil.rmtree(latest_dst)
    shutil.copytree(latest_src, latest_dst)

    # Keep only history tags (exclude latest tag since latest/ already points to it).
    for child in OUTPUT_DIR.iterdir():
        if child.is_dir() and child.name != "latest" and child not in history_tag_dirs:
            shutil.rmtree(child)

    # Make root URL useful on GitHub Pages by forwarding to latest.
    (OUTPUT_DIR / "index.html").write_text(
        "<!doctype html><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0; url=latest/\">",
        encoding="utf-8",
    )

    # Keep versions.json simple for the custom switcher.
    (OUTPUT_DIR / "versions.json").write_text(
        dumps(
            [{"name": "latest", "label": f"latest ({latest_src.name})"}]
            + [{"name": tag.name, "label": tag.name} for tag in reversed(history_tag_dirs)],
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    # Force latest switcher assets/styles into every built tag snapshot.
    source_switcher_js = ROOT / "source" / "_static" / "version-switcher.js"
    for built_dir in OUTPUT_DIR.iterdir():
        if not built_dir.is_dir() or (built_dir.name != "latest" and not TAG_RE.match(built_dir.name)):
            continue
        static_dir = built_dir / "_static"
        if not static_dir.exists():
            continue

        if source_switcher_js.exists():
            shutil.copy2(source_switcher_js, static_dir / "version-switcher.js")

        css_path = static_dir / "custom.css"
        existing_css = css_path.read_text(encoding="utf-8") if css_path.exists() else ""
        if "Polyversion runtime override" not in existing_css:
            if existing_css and not existing_css.endswith("\n"):
                existing_css += "\n"
            css_path.write_text(existing_css + "\n" + STYLE_OVERRIDE + "\n", encoding="utf-8")
