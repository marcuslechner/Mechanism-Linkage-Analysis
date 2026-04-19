# Converting Markdown to PDF with fpdf2

A step-by-step guide to what was done to convert `6bar6_1_dimensions.md` into a styled PDF using only the project's existing Python virtual environment.

---

## 1. Why fpdf2?

The first choice was `weasyprint`, which converts HTML+CSS to PDF and handles Unicode well. It failed because it requires the GTK/Pango native libraries (`libgobject-2.0-0`), which are not installed on this machine.

`fpdf2` has no native dependencies — it is pure Python and was already installed in `.venv`. The trade-off is that you must render every element yourself instead of relying on a CSS engine.

---

## 2. Install (already done)

```
.venv/Scripts/pip install fpdf2
```

`fpdf2` pulls in `Pillow` and `fonttools` but nothing that requires system libraries.

---

## 3. The encoding problem

fpdf2's built-in fonts (Helvetica, Courier, Times) are **latin-1 only**. The markdown file contains Unicode characters that are not in latin-1:

| Character | Unicode | Fix |
|---|---|---|
| en-dash – | U+2013 | replace with `-` |
| arrow → | U+2192 | replace with `->` |
| multiplication × | U+00D7 | replace with `x` |
| delta Δ | U+0394 | replace with `delta` |

The fix was a `clean()` function that replaces known offenders, then encodes to latin-1 with `errors='replace'` as a final safety net:

```python
def clean(text):
    return (text
        .replace('\u2013', '-')
        .replace('\u2192', '->')
        # ... etc
    ).encode('latin-1', errors='replace').decode('latin-1')
```

The alternative is to load a Unicode TTF font with `pdf.add_font()`, which lets you skip all of this — but requires shipping a `.ttf` file.

---

## 4. Document setup

```python
from fpdf import FPDF
from fpdf.enums import XPos, YPos

pdf = FPDF()
pdf.add_page()
pdf.set_margins(20, 20, 20)   # left, top, right in mm
pdf.set_auto_page_break(auto=True, margin=20)
```

`set_auto_page_break` means fpdf2 will automatically start a new page when content reaches 20 mm from the bottom.

---

## 5. Parsing the markdown

There is no markdown parser involved — the file is read line by line and each line is classified by its prefix:

| Prefix | Element |
|---|---|
| `# ` | H1 heading |
| `## ` | H2 heading |
| `### ` | H3 heading |
| `` ``` `` | Start/end of a code block |
| `\|` | A table row (buffered until a non-table line) |
| `---` or blank | Vertical space |
| anything else | Normal paragraph text |

The loop maintains two state variables:
- `in_code` — True while inside a fenced code block
- `table_buf` — list of raw table lines being accumulated

Tables are buffered because you need all rows before you can draw them (to know column count and widths). When the first non-table line is encountered, the buffer is flushed to `draw_table()`.

---

## 6. Rendering each element

### Headings

Each heading level uses a larger font, a different blue shade, and a horizontal rule drawn underneath with `pdf.line()`:

```python
pdf.set_font('Helvetica', 'B', 16)          # H1: 16pt bold
pdf.set_text_color(10, 40, 100)             # dark blue
pdf.cell(0, 10, text, new_x=XPos.LMARGIN, new_y=YPos.NEXT)
pdf.set_line_width(0.8)
pdf.line(pdf.l_margin, pdf.get_y(),         # draw rule across page
         pdf.w - pdf.r_margin, pdf.get_y())
```

H2 uses 13pt, a slightly lighter blue, and a thinner rule (0.5 mm). H3 uses 11pt with no rule.

### Tables

```python
def parse_table_rows(raw_rows):
    # skip separator rows like |---|---|
    # split each row on | and strip whitespace
```

Column widths are hardcoded for the two table shapes in this document (5-column link table, 3-column pivot table). For a general solution you would measure the widest cell in each column.

Each cell is drawn with `pdf.cell()`:

```python
pdf.set_fill_color(*HEAD_BG)   # shaded background for header
pdf.cell(w, 7, cell_text,
         border=1,             # draw all four borders
         align='R',            # right-align numeric columns
         fill=True,
         new_x=XPos.RIGHT,     # cursor moves right after cell
         new_y=YPos.TOP)       # cursor stays on same row
pdf.ln()                       # move to next row after all cells
```

Alternating row fill uses `ri % 2 == 1` to pick between white and a light blue-grey.

### Code blocks

Lines between `` ``` `` fences are collected, then rendered in Courier 8.5pt with a grey background fill. Each line is one `pdf.cell()` call with `fill=True` and no border.

### Inline formatting

`render_inline()` splits the line on `**bold**` and `` `code` `` spans using a regex, then switches fonts for each segment:

```python
parts = re.split(r'(\*\*[^*]+\*\*|`[^`]+`)', text)
for part in parts:
    if part.startswith('**'):
        pdf.set_font('Helvetica', 'B', 10)
        pdf.write(5, clean(part[2:-2]))
        pdf.set_font('Helvetica', '', 10)   # reset
    elif part.startswith('`'):
        pdf.set_font('Courier', '', 9)
        pdf.write(5, clean(part[1:-1]))
        pdf.set_font('Helvetica', '', 10)
    else:
        pdf.write(5, clean(part))
```

`pdf.write()` unlike `pdf.cell()` does not advance to a new line — it flows text inline, so multiple calls on the same logical line stay on the same PDF line.

---

## 7. Output

```python
pdf.output("path/to/output.pdf")
```

That is the entire pipeline — no intermediate HTML, no external tools.

---

## 8. Limitations of this approach

- **No word wrap** — lines longer than the page width are clipped. The markdown in this document has no long prose paragraphs so this was not a problem.
- **No Unicode** — requires the `clean()` workaround. Loading a TTF font solves this permanently.
- **Hardcoded column widths** — fine for a known document shape, brittle for arbitrary markdown.
- **No nested formatting** — e.g. `**bold `code`**` would not render correctly.

For a more robust general solution: pipe the markdown through the `markdown` library to get HTML, then use a tool like `weasyprint` (once GTK is installed) or `playwright` to render it.

---

## 9. Reuse on another file

To run this on a different markdown file, change the two path variables at the top of the script:

```python
md_path = "../CAD/Reference/your_file.md"
pdf_path = "../CAD/Reference/your_file.pdf"
```

Then run:

```
.venv/Scripts/python your_script.py
```
