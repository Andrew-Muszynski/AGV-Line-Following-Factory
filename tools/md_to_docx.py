#!/usr/bin/env python3
"""Render the progress report Markdown as a professionally formatted .docx.

Written because neither pandoc nor LibreOffice is installed on either machine
(checked 2026-10-03), so the usual Markdown -> Word path is unavailable.
python-docx is, and gives finer control over the result than a pandoc
reference document would anyway.

Deliberately NOT a general Markdown converter. It handles exactly the
constructs this report uses -- ATX headings, pipe tables with escaped pipes,
fenced code, bullets, blockquotes, inline bold/italic/code/links, and
horizontal rules -- and will quietly pass anything else through as text. That
is the right trade for a document whose source is known; a general converter
would be far more code and no more correct here.

    python3 tools/md_to_docx.py PROGRESS_REPORT_2026-10.md -o report.docx
"""
from __future__ import annotations

import argparse
import itertools
import re
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor

# --- palette -----------------------------------------------------------
# One accent, used for headings and table header fill. A technical report
# does not need a second.
NAVY = RGBColor(0x1F, 0x35, 0x53)
SLATE = RGBColor(0x44, 0x4A, 0x52)
RULE = "BFC6D1"
HEADER_FILL = "1F3553"
BAND_FILL = "F2F4F7"
CODE_FILL = "F4F5F7"

# Density profile. The same source renders as a full report or as a
# page-limited brief; only spacing and type size differ, never content.
COMPACT = False

BODY_FONT = "Cambria"       # serif body: this is read, not skimmed
HEAD_FONT = "Calibri Light"
MONO_FONT = "Consolas"


# --- low-level helpers --------------------------------------------------

def _shade(el, fill: str) -> None:
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")        # never "solid" -- renders black
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    el.append(shd)


def _borders(par, *, bottom: str | None = None, size: int = 6) -> None:
    pPr = par._p.get_or_add_pPr()
    bdr = OxmlElement("w:pBdr")
    if bottom:
        b = OxmlElement("w:bottom")
        b.set(qn("w:val"), "single")
        b.set(qn("w:sz"), str(size))
        b.set(qn("w:space"), "4")
        b.set(qn("w:color"), bottom)
        bdr.append(b)
    pPr.append(bdr)


def _keep_with_next(par) -> None:
    pPr = par._p.get_or_add_pPr()
    k = OxmlElement("w:keepNext")
    pPr.append(k)


def _repeat_header(row) -> None:
    trPr = row._tr.get_or_add_trPr()
    h = OxmlElement("w:tblHeader")
    trPr.append(h)


def _field(par, instr: str) -> None:
    """A Word field, so the TOC and page numbers stay live."""
    r = par.add_run()
    fc = OxmlElement("w:fldChar"); fc.set(qn("w:fldCharType"), "begin")
    it = OxmlElement("w:instrText"); it.set(qn("xml:space"), "preserve")
    it.text = instr
    sep = OxmlElement("w:fldChar"); sep.set(qn("w:fldCharType"), "separate")
    end = OxmlElement("w:fldChar"); end.set(qn("w:fldCharType"), "end")
    for e in (fc, it, sep, end):
        r._r.append(e)


# --- inline markdown ----------------------------------------------------

INLINE = re.compile(
    r"(\*\*.+?\*\*"            # bold
    r"|`[^`]+`"                # code
    r"|\*[^*\n]+?\*"           # italic
    r"|\[[^\]]+\]\([^)]+\))")  # link -> text only


def add_inline(par, text, *, size=None, color=None, bold=False,
               italic=False, _top=True):
    """Markdown inline spans into runs, RECURSIVELY.

    Nesting is the whole reason this recurses. `**`tools/x.py`**`
    -- bold wrapping code, which this report uses throughout its tool lists
    -- was previously handled by stripping the asterisks and emitting the
    rest as one run, so the backticks survived as literal text in the Word
    file. Caught by reading the generated document back.

    Links keep their text and lose the target: every link in this document
    is an intra-document anchor, which means nothing once it is a .docx.
    """
    if _top:
        text = text.replace(r"\|", "|").replace(r"\*", "*")

    def style(run, *, mono=False):
        run.bold = bold or None
        run.italic = italic or None
        run.font.name = MONO_FONT if mono else BODY_FONT
        if size:
            run.font.size = Pt(size - (1.0 if mono else 0.0))
        elif mono:
            run.font.size = Pt(9.5)
        if color is not None:
            run.font.color.rgb = color
        return run

    for piece in INLINE.split(text):
        if not piece:
            continue
        if piece.startswith("**") and piece.endswith("**") and len(piece) > 4:
            add_inline(par, piece[2:-2], size=size, color=color,
                       bold=True, italic=italic, _top=False)
        elif piece.startswith("`") and piece.endswith("`") and len(piece) > 2:
            r = style(par.add_run(piece[1:-1]), mono=True)
            _shade(r._r.get_or_add_rPr(), CODE_FILL)
        elif (piece.startswith("*") and piece.endswith("*")
              and len(piece) > 2):
            add_inline(par, piece[1:-1], size=size, color=color,
                       bold=bold, italic=True, _top=False)
        elif piece.startswith("["):
            m = re.match(r"\[([^\]]+)\]\([^)]+\)", piece)
            add_inline(par, m.group(1) if m else piece, size=size,
                       color=color, bold=bold, italic=italic, _top=False)
        else:
            style(par.add_run(piece))
    return par


# --- document scaffolding ----------------------------------------------

def build_styles(doc: Document) -> None:
    st = doc.styles["Normal"]
    st.font.name = BODY_FONT
    st.font.size = Pt(9.0 if COMPACT else 10.5)
    st.paragraph_format.space_after = Pt(3 if COMPACT else 7)
    st.paragraph_format.line_spacing = 1.0 if COMPACT else 1.12

    heads = (("Heading 1", 12.5, NAVY, 9, 3),
             ("Heading 2", 10.5, NAVY, 8, 2),
             ("Heading 3", 9.5, SLATE, 6, 1)) if COMPACT else (
            ("Heading 1", 17, NAVY, 22, 8),
            ("Heading 2", 13, NAVY, 16, 6),
            ("Heading 3", 11, SLATE, 12, 4))
    for name, size, color, before, after in heads:
        s = doc.styles[name]
        s.font.name = HEAD_FONT
        s.font.size = Pt(size)
        s.font.color.rgb = color
        s.font.bold = (name == "Heading 3")
        s.paragraph_format.space_before = Pt(before)
        s.paragraph_format.space_after = Pt(after)
        s.paragraph_format.keep_with_next = True


def page_setup(doc: Document) -> None:
    for s in doc.sections:
        s.page_width, s.page_height = Inches(8.5), Inches(11)   # US Letter
        m = 0.7 if COMPACT else 1.0
        s.left_margin = s.right_margin = Inches(m)
        s.top_margin = Inches(0.6 if COMPACT else 0.9)
        s.bottom_margin = Inches(0.5 if COMPACT else 0.9)
        # Footer: title left, page number right, on a dot leader.
        p = s.footer.paragraphs[0]
        p.paragraph_format.tab_stops.add_tab_stop(
            Inches(6.5), WD_TAB_ALIGNMENT.RIGHT)
        r = p.add_run("AGV testbed — progress report\t")
        r.font.size = Pt(8.5)
        r.font.color.rgb = SLATE
        r.font.name = HEAD_FONT
        _field(p, " PAGE ")
        for run in p.runs:
            run.font.size = Pt(8.5)
            run.font.color.rgb = SLATE


def title_block(doc: Document, title: str, meta: list[str]) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run(title)
    r.font.name = HEAD_FONT
    r.font.size = Pt(16 if COMPACT else 23)
    r.font.bold = True
    r.font.color.rgb = NAVY

    for line in meta:
        q = doc.add_paragraph()
        q.paragraph_format.space_after = Pt(1)
        add_inline(q, line, size=9.5, color=SLATE)
    _borders(doc.paragraphs[-1], bottom=RULE, size=8)
    doc.paragraphs[-1].paragraph_format.space_after = Pt(16)


def toc(doc: Document) -> None:
    h = doc.add_paragraph()
    r = h.add_run("Contents")
    r.font.name = HEAD_FONT
    r.font.size = Pt(13)
    r.font.bold = True
    r.font.color.rgb = NAVY
    h.paragraph_format.space_after = Pt(6)
    p = doc.add_paragraph()
    _field(p, r' TOC \o "1-2" \h \z \u ')
    # The TOC field ships EMPTY. python-docx can write the field code but
    # cannot compute the entries -- only Word can walk the headings and
    # resolve page numbers. tools/populate_fields.ps1 does that and saves,
    # so the delivered file carries a real contents list rather than a line
    # telling the reader to refresh it.
    doc.add_page_break()


# --- block parsing ------------------------------------------------------

def split_row(line: str) -> list[str]:
    cells, cur, esc = [], "", False
    for ch in line.strip().strip("|"):
        if esc:
            cur += ch
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == "|":
            cells.append(cur.strip())
            cur = ""
        else:
            cur += ch
    cells.append(cur.strip())
    return cells


def add_table(doc: Document, rows: list[list[str]]) -> None:
    cols = max(len(r) for r in rows)
    t = doc.add_table(rows=0, cols=cols)
    t.style = "Table Grid"
    if COMPACT:
        mar = OxmlElement("w:tblCellMar")
        for side, val in (("top", 6), ("bottom", 6), ("left", 60), ("right", 60)):
            e = OxmlElement(f"w:{side}")
            e.set(qn("w:w"), str(val))
            e.set(qn("w:type"), "dxa")
            mar.append(e)
        t._tbl.tblPr.append(mar)
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = True
    for i, cells in enumerate(rows):
        row = t.add_row()
        if i == 0:
            _repeat_header(row)
        for j in range(cols):
            cell = row.cells[j]
            cell.paragraphs[0].text = ""
            par = cell.paragraphs[0]
            par.paragraph_format.space_before = Pt(0 if COMPACT else 2)
            par.paragraph_format.space_after = Pt(0 if COMPACT else 2)
            par.paragraph_format.line_spacing = 1.0
            txt = cells[j] if j < len(cells) else ""
            add_inline(par, txt, size=7.5 if COMPACT else 9,
                       color=RGBColor(0xFF, 0xFF, 0xFF) if i == 0 else None,
                       bold=(i == 0))
            _shade(cell._tc.get_or_add_tcPr(),
                   HEADER_FILL if i == 0 else (BAND_FILL if i % 2 == 0 else "FFFFFF"))
    doc.add_paragraph().paragraph_format.space_after = Pt(4)


def add_code(doc: Document, lines: list[str]) -> None:
    for k, line in enumerate(lines):
        p = doc.add_paragraph()
        pf = p.paragraph_format
        pf.left_indent = Inches(0.22)
        pf.space_before = Pt(5 if k == 0 else 0)
        pf.space_after = Pt(5 if k == len(lines) - 1 else 0)
        pf.line_spacing = 1.0
        r = p.add_run(line if line.strip() else " ")
        r.font.name = MONO_FONT
        r.font.size = Pt(7.5 if COMPACT else 8.5)
        _shade(p._p.get_or_add_pPr(), CODE_FILL)


BLOCK_START = re.compile(r"^(#{1,4}\s|\||```|>\s|[-*]\s|\d+\.\s|-{3,}$)")


def add_figure(doc: Document, path: Path, caption: str, number: int) -> None:
    """Embed a figure, scaled to the text column, with a caption beneath.

    Width is clamped to the live text width rather than the paper width --
    a 2070px chart inserted at native size runs into the margins and Word
    will not fix it for you. Height follows from the image's own aspect, so
    nothing is stretched.
    """
    from PIL import Image

    sec = doc.sections[0]
    avail = sec.page_width - sec.left_margin - sec.right_margin
    w, h = Image.open(path).size

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(2)
    _keep_with_next(p)
    p.add_run().add_picture(str(path), width=avail)

    cap = doc.add_paragraph()
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.space_after = Pt(12)
    add_inline(cap, f"**Figure {number}.** {caption}", size=Pt(9), color=SLATE)
    for r in cap.runs:
        r.font.name = BODY_FONT


def gather_item(lines, i, first):
    """A list item plus its wrapped continuation lines.

    Taking only the first line splits an item mid-sentence, and worse,
    splits inline markup across two paragraphs so the markers survive as
    literal text -- `**Two grader defects ... rather than` in one paragraph
    and `summaries.**` in the next. Caught on the first render.
    """
    buf = [first]
    i += 1
    while i < len(lines):
        nxt = lines[i]
        if not nxt.strip():
            break
        # a continuation is indented and does not open a new block
        if not nxt.startswith((" ", "	")):
            break
        if BLOCK_START.match(nxt.strip()):
            break
        buf.append(nxt.strip())
        i += 1
    return " ".join(buf), i


def convert(md: str, out: Path, base: Path | None = None) -> None:
    """Render `md` to `out`.

    `base` is the directory that relative image paths resolve against -- the
    markdown file's own directory, which is what Markdown means by a relative
    link. It used to resolve against the output directory, which only agreed
    with Markdown while source and output sat side by side; once the sources
    moved to docs/ and the .docx files to THESIS PAPER/ it silently looked in
    the wrong place. Defaults to the output directory so older callers that
    pass only two arguments keep working.
    """
    base = base or out.parent
    doc = Document()
    build_styles(doc)
    page_setup(doc)

    fignum = itertools.count(1)
    lines = md.split("\n")
    i, n = 0, len(lines)

    # --- title block -----------------------------------------------------
    title = lines[0].lstrip("# ").strip()
    i = 1
    # The metadata block ends at the first rule OR the first heading.
    # Terminating only on "---" silently swallowed an entire document whose
    # sections are not rule-separated: every line became a metadata line, so
    # the output had 350 paragraphs and no tables at all.
    meta = []
    while i < n and not lines[i].startswith("---")             and not lines[i].startswith("#"):
        if lines[i].strip():
            meta.append(lines[i].strip())
        i += 1
    title_block(doc, title, meta)
    if not COMPACT:
        toc(doc)

    # Skip a source Contents list if present -- the live TOC replaces it.
    # Only skip forward if a "## 1." heading actually exists ahead, so a
    # document without one is not consumed entirely.
    first = next((k for k in range(i, n)
                  if re.match(r"^## 1\.", lines[k])), None)
    if first is not None:
        i = first

    while i < n:
        line = lines[i]
        stripped = line.strip()

        if not stripped or re.fullmatch(r"-{3,}", stripped):
            i += 1
            continue

        if stripped.startswith("```"):
            i += 1
            block = []
            while i < n and not lines[i].strip().startswith("```"):
                block.append(lines[i])
                i += 1
            i += 1
            add_code(doc, block)
            continue

        if stripped.startswith("|") and i + 1 < n and re.match(
                r"^\|[\s:|-]+\|$", lines[i + 1].strip()):
            rows = [split_row(stripped)]
            i += 2
            while i < n and lines[i].strip().startswith("|"):
                rows.append(split_row(lines[i].strip()))
                i += 1
            add_table(doc, rows)
            continue

        # Figure: ![caption](figures/x.png) alone on a line. Numbered in
        # source order and captioned beneath, which is what a reader expects
        # and what lets body text refer to "Figure 3" without a live field.
        m = re.match(r"^!\[(.*?)\]\((.+?)\)$", stripped)
        if m:
            caption, src = m.group(1), (base / m.group(2))
            if not src.exists():
                raise SystemExit(f"figure not found: {src}")
            add_figure(doc, src, caption, next(fignum))
            i += 1
            continue

        m = re.match(r"^(#{1,4})\s+(.*)$", stripped)
        if m:
            lvl = min(len(m.group(1)), 3)
            txt = re.sub(r"^\d+(\.\d+)*\.?\s+", "", m.group(2)).strip()
            h = doc.add_heading(level=lvl)
            add_inline(h, txt)
            for r in h.runs:
                r.font.name = HEAD_FONT
                r.font.color.rgb = NAVY if lvl < 3 else SLATE
                r.font.bold = (lvl == 3)
            i += 1
            continue

        if stripped.startswith("> "):
            p = doc.add_paragraph()
            p.paragraph_format.left_indent = Inches(0.3)
            p.paragraph_format.space_before = Pt(4)
            p.paragraph_format.space_after = Pt(8)
            add_inline(p, stripped[2:], color=SLATE)
            for r in p.runs:
                r.italic = True
            _borders(p, bottom=RULE, size=4)
            i += 1
            continue

        m = re.match(r"^(\d+)\.\s+(.*)$", stripped)
        if m:
            body, i = gather_item(lines, i, m.group(2))
            add_inline(doc.add_paragraph(style="List Number"), body)
            continue

        if re.match(r"^[-*]\s+", stripped):
            body, i = gather_item(
                lines, i, re.sub(r"^[-*]\s+", "", stripped))
            add_inline(doc.add_paragraph(style="List Bullet"), body)
            continue

        # paragraph: join wrapped source lines
        buf = [stripped]
        i += 1
        while i < n and lines[i].strip() and not re.match(
                r"^(#{1,4}\s|\||```|>\s|[-*]\s|\d+\.\s|-{3,}$)",
                lines[i].strip()):
            buf.append(lines[i].strip())
            i += 1
        add_inline(doc.add_paragraph(), " ".join(buf))

    n_tbl = len(doc.tables)
    n_head = sum(1 for p in doc.paragraphs
                 if p.style.name.startswith("Heading"))
    want_tbl = sum(1 for l in lines if re.match(r"^\|[\s:|-]+\|$", l.strip()))
    want_head = sum(1 for l in lines if re.match(r"^#{2,3}\s", l))
    if n_tbl != want_tbl or n_head < want_head - 1:
        raise SystemExit(
            f"conversion incomplete: produced {n_tbl} of {want_tbl} tables "
            f"and {n_head} of {want_head} headings. Refusing to write a "
            f"silently truncated document.\n"
            f"  If the table count is short by one or two, check for a header "
            f"row of empty cells such as '| | |'. It matches the separator "
            f"pattern, so it is counted as a table that was never built. "
            f"Give every table real column headers.")
    doc.save(str(out))
    return doc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("source")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--compact", action="store_true",
                    help="page-limited brief: tighter type and spacing, no "
                         "contents page. Content is identical.")
    a = ap.parse_args()
    global COMPACT
    COMPACT = a.compact
    src = Path(a.source)
    out = Path(a.out) if a.out else src.with_suffix(".docx")
    convert(src.read_text(encoding="utf-8"), out, src.parent)
    print(f"wrote {out}  ({out.stat().st_size:,} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
