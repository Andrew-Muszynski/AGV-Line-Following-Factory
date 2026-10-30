# -*- coding: utf-8 -*-
"""Fill the URI thesis template with the draft in thesis_content.py."""
import copy
import sys
from pathlib import Path

import docx
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_COLOR_INDEX
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt

sys.path.insert(0, str(Path(__file__).parent))
import thesis_content as C  # noqa: E402

SRC = Path(r"C:\Users\muszy\VRP\THESIS PAPER\thesis_template_March2026 (1).docx")
OUT = Path(r"C:\Users\muszy\VRP\THESIS PAPER\Muszynski_Thesis_draft_v1.docx")

d = docx.Document(SRC)
P = list(d.paragraphs)                 # original indices, captured once


def has_sect(p):
    ppr = p._p.find(qn("w:pPr"))
    return ppr is not None and ppr.find(qn("w:sectPr")) is not None


def set_text(p, text):
    """Replace a paragraph's text, keeping its first run's formatting."""
    runs = p.runs
    if not runs:
        p.add_run(text)
        return
    runs[0].text = text
    for r in runs[1:]:
        r._r.getparent().remove(r._r)


def clear(p):
    for r in list(p.runs):
        r._r.getparent().remove(r._r)
    for h in p._p.findall(qn("w:hyperlink")):
        p._p.remove(h)


def remove(p):
    p._p.getparent().remove(p._p)


# ---- styles --------------------------------------------------------------
H1 = P[111].style
H2 = P[113].style


def make_style(name, size, bold, italic=False, outline=None, align=None,
               before=12, after=6, keep=True):
    try:
        return d.styles[name]
    except KeyError:
        pass
    s = d.styles.add_style(name, WD_STYLE_TYPE.PARAGRAPH)
    s.base_style = d.styles["Normal"]
    s.font.name, s.font.size, s.font.bold = "Arial", Pt(size), bold
    s.font.italic = italic
    pf = s.paragraph_format
    pf.space_before, pf.space_after = Pt(before), Pt(after)
    pf.keep_with_next = keep
    if align is not None:
        pf.alignment = align
    if outline is not None:
        ppr = s.element.get_or_add_pPr()
        ol = OxmlElement("w:outlineLvl")
        ol.set(qn("w:val"), str(outline))
        ppr.append(ol)
    return s


H3 = make_style("Thesis Heading 3", 12, True, outline=2)
H4 = make_style("Thesis Heading 4", 12, True, italic=True, outline=3)
BODY = make_style("Thesis Body", 12, False, before=0, after=0, keep=False)
BODY.paragraph_format.first_line_indent = Inches(0.5)
BODY.paragraph_format.line_spacing = 2.0
LEAD = make_style("Thesis List Item", 12, False, before=0, after=0,
                  keep=False)
LEAD.paragraph_format.left_indent = Inches(0.5)
LEAD.paragraph_format.first_line_indent = Inches(-0.25)
LEAD.paragraph_format.line_spacing = 2.0
FIGCAP = make_style("Figure Caption", 11, False, align=WD_ALIGN_PARAGRAPH.CENTER,
                    before=4, after=14, keep=False)
TABCAP = make_style("Table Caption", 11, False, before=12, after=4)
CELL = make_style("Table Text", 10, False, before=0, after=0, keep=False)
BIBS = make_style("Bibliography Entry", 12, False, before=0, after=12,
                  keep=False)
BIBS.paragraph_format.left_indent = Inches(0.5)
BIBS.paragraph_format.first_line_indent = Inches(-0.5)


# ---- builders ------------------------------------------------------------
class Writer:
    def __init__(self, anchor):
        self.last = anchor._p

    def _place(self, el):
        self.last.addnext(el)
        self.last = el

    def para(self, text="", style=BODY):
        p = d.add_paragraph(style=style)
        if text:
            p.add_run(text)
        self._place(p._p)
        return p

    def note(self, text):
        p = d.add_paragraph(style=BODY)
        r = p.add_run("[ADD: " + text + "]")
        r.italic = True
        r.font.highlight_color = WD_COLOR_INDEX.YELLOW
        self._place(p._p)

    def lead(self, term, text):
        p = d.add_paragraph(style=LEAD)
        p.add_run(term + " ").bold = True
        p.add_run(text)
        self._place(p._p)

    def figure(self, path, caption, width):
        p = d.add_paragraph(style=BODY)
        p.paragraph_format.first_line_indent = Inches(0)
        p.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.keep_with_next = True
        p.paragraph_format.line_spacing = 1.0
        p.add_run().add_picture(path, width=Inches(width))
        self._place(p._p)
        self.para(caption, FIGCAP)

    def table(self, caption, header, rows):
        self.para(caption, TABCAP)
        t = d.add_table(rows=1 + len(rows), cols=len(header))
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        borders(t)
        for j, h in enumerate(header):
            c = t.rows[0].cells[j]
            c.paragraphs[0].style = CELL
            c.paragraphs[0].add_run(h).bold = True
            shade(c, "D9D9D9")
        for i, row in enumerate(rows, 1):
            for j, v in enumerate(row):
                c = t.rows[i].cells[j]
                c.paragraphs[0].style = CELL
                c.paragraphs[0].add_run(v)
        for i, row in enumerate(t.rows):        # keep the table on one page
            trPr = row._tr.get_or_add_trPr()
            cs = OxmlElement("w:cantSplit")
            cs.set(qn("w:val"), "true")
            trPr.append(cs)
            if i < len(t.rows) - 1:
                for c in row.cells:
                    c.paragraphs[0].paragraph_format.keep_with_next = True
        hdr = t.rows[0]._tr.get_or_add_trPr()
        th = OxmlElement("w:tblHeader")
        th.set(qn("w:val"), "true")
        hdr.append(th)
        self._place(t._tbl)
        self.para("", CELL)          # spacer after the table


def borders(t):
    tblPr = t._tbl.tblPr
    b = OxmlElement("w:tblBorders")
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        e = OxmlElement(f"w:{edge}")
        e.set(qn("w:val"), "single")
        e.set(qn("w:sz"), "4")
        e.set(qn("w:space"), "0")
        e.set(qn("w:color"), "808080")
        b.append(e)
    tblPr.append(b)


def shade(cell, fill):
    tcPr = cell._tc.get_or_add_tcPr()
    s = OxmlElement("w:shd")
    s.set(qn("w:val"), "clear")
    s.set(qn("w:color"), "auto")
    s.set(qn("w:fill"), fill)
    tcPr.append(s)


def field(p, instr, placeholder):
    """Insert a complex field (TOC etc.) that Word fills on update."""
    def run(child):
        r = OxmlElement("w:r")
        r.append(child)
        p._p.append(r)
    b = OxmlElement("w:fldChar")
    b.set(qn("w:fldCharType"), "begin")
    run(b)
    it = OxmlElement("w:instrText")
    it.set(qn("xml:space"), "preserve")
    it.text = instr
    run(it)
    s = OxmlElement("w:fldChar")
    s.set(qn("w:fldCharType"), "separate")
    run(s)
    t = OxmlElement("w:t")
    t.text = placeholder
    run(t)
    e = OxmlElement("w:fldChar")
    e.set(qn("w:fldCharType"), "end")
    run(e)


def write_blocks(w, blocks, chap):
    """chap: chapter number or letter, for figure/table numbering."""
    nf = nt = 0
    for b in blocks:
        k = b[0]
        if k == "h3":
            w.para(b[1], H3)
        elif k == "h4":
            w.para(b[1], H4)
        elif k == "p":
            w.para(b[1])
        elif k == "lead":
            w.lead(b[1], b[2])
        elif k == "note":
            w.note(b[1])
        elif k == "fig":
            nf += 1
            w.figure(b[1], f"Figure {chap}.{nf}. {b[2]}", b[3])
        elif k == "table":
            nt += 1
            w.table(f"Table {chap}.{nt}. {b[1]}", b[2], b[3])


# ---- title and approval pages -------------------------------------------
set_text(P[0], C.TITLE)
set_text(P[2], "ANDREW MUSZYNSKI")
set_text(P[7], "A THESIS SUBMITTED IN PARTIAL FULFILLMENT OF THE")
set_text(P[9], "MASTER OF SCIENCE")
set_text(P[11], "INDUSTRIAL AND SYSTEMS ENGINEERING")
set_text(P[18], "2027")
set_text(P[19], "MASTER OF SCIENCE THESIS")
P[19].paragraph_format.page_break_before = True
set_text(P[23], "ANDREW MUSZYNSKI")
set_text(P[37], "Thesis Committee:")
set_text(P[39], "Major Professor\tManbir Sodhi")
set_text(P[57], "2027")

# ---- abstract, acknowledgments, preface ---------------------------------
def fill_front(title_idx, first, last_sect, blocks):
    """Replace placeholder paragraphs first..last_sect (keep the section
    break paragraph, emptied) with blocks."""
    w = Writer(P[title_idx])
    for b in blocks:
        if b[0] == "p":
            w.para(b[1])
        else:
            w.note(b[1])
    for i in range(first, last_sect):
        remove(P[i])
    clear(P[last_sect])


fill_front(58, 59, 66, C.ABSTRACT)
fill_front(67, 68, 72, C.ACK)
remove_from = 72
fill_front(73, 74, 81, C.PREFACE)

# ---- table of contents and lists ----------------------------------------
# The template already carries a TOC field (in a content control); Word
# refreshes it on open, so nothing is inserted here.
from docx.enum.text import WD_TAB_ALIGNMENT
for i, label in ((88, "TABLE"), (101, "FIGURE")):
    set_text(P[i], label + "\tPAGE")
    P[i].paragraph_format.tab_stops.clear_all()
    P[i].paragraph_format.tab_stops.add_tab_stop(Inches(6.0),
                                                 WD_TAB_ALIGNMENT.RIGHT)
# list of tables: keep the header row, one TOC field over Table Caption
clear(P[89])
field(P[89], ' TOC \\h \\z \\t "Table Caption,1" ',
      "Right-click and choose Update Field to build the list of tables.")
for i in (90, 91, 92, 93, 94, 95, 96, 97):
    remove(P[i])
clear(P[98])
clear(P[102])
field(P[102], ' TOC \\h \\z \\t "Figure Caption,1" ',
      "Right-click and choose Update Field to build the list of figures.")
for i in (103, 104, 105, 106, 107, 108, 109):
    remove(P[i])
clear(P[110])

# ---- chapters -------------------------------------------------------------
# Template chapter blocks: (Heading 1 idx, Heading 2 idx, body idxs, last sect)
blocks = [(111, 113, [114, 115, 116, 117, 118], 119),
          (120, 122, [123, 124, 125], 126),
          (127, 129, [130, 131, 132], 133),
          (134, 136, [137, 138, 139], 140),
          (141, 143, [144, 145, 146], 147)]

# A sixth chapter: clone chapter 5's heading pair and closing break.
h1c = copy.deepcopy(P[141]._p)
blank1 = copy.deepcopy(P[142]._p)
h2c = copy.deepcopy(P[143]._p)
blank2 = copy.deepcopy(P[144]._p)
endc = copy.deepcopy(P[147]._p)
anchor = P[147]._p
for el in (h1c, blank1, h2c, blank2, endc):
    anchor.addnext(el)
    anchor = el
P6 = [docx.text.paragraph.Paragraph(e, P[147]._parent)
      for e in (h1c, blank1, h2c, blank2, endc)]

for n, ((title, content), spec) in enumerate(
        zip(C.CHAPTERS, blocks + [None]), 1):
    if spec is None:
        h1, h2, endp = P6[0], P6[2], P6[4]
        body = []
        anchor_p = P6[3]
    else:
        h1, h2, body, endp = P[spec[0]], P[spec[1]], \
            [P[i] for i in spec[2]], P[spec[3]]
        anchor_p = body[0]           # the blank under the Heading 2
        body = body[1:]
    set_text(h1, f"CHAPTER {n}")
    set_text(h2, title)
    clear(endp)
    w = Writer(anchor_p)
    write_blocks(w, content, n)
    for p in body:
        remove(p)

# ---- appendices -----------------------------------------------------------
set_text(P[150], "")
w = Writer(P[150])
nt = 0
for b in C.APPENDICES:
    if b[0] == "h3":
        w.para(b[1], H3)
        letter = b[1].split()[1].rstrip(":")
        nt = 0
    elif b[0] == "note":
        w.note(b[1])
    elif b[0] == "table":
        nt += 1
        w.table(f"Table {letter}.{nt}. {b[1]}", b[2], b[3])

# ---- bibliography ---------------------------------------------------------
w = Writer(P[157])
w.note("Confirm the citation style with the Graduate School and advisor. "
       "The text uses author-year citations; entries are alphabetical. "
       "Check every entry against its source.")
for ref in C.BIB:
    w.para(ref, BIBS)
for i in (158, 159, 160, 161, 162):
    remove(P[i])

# ---- ask Word to refresh fields (TOC, lists) on open ----------------------
settings = d.settings.element
uf = OxmlElement("w:updateFields")
uf.set(qn("w:val"), "true")
settings.append(uf)

d.save(OUT)
print(OUT)
