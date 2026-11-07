"""Write Drawings/RV-100_BOM.xlsx from the CSV that bom.py writes (needs openpyxl, no SolidWorks).

    python bom_xlsx.py
"""
import csv
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CSV = os.path.join(ROOT, "Drawings", "RV-100_BOM.csv")
NUMERIC = {"Qty", "Unit_mass_g", "Total_mass_g"}


def write_xlsx(csv_path=CSV):
    from openpyxl import Workbook
    from openpyxl.styles import Alignment, Font, PatternFill
    from openpyxl.utils import get_column_letter

    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    wb = Workbook()
    ws = wb.active
    ws.title = "BOM"
    ws.append(head)
    for r in body:
        ws.append([float(v) if h in NUMERIC and v not in ("", None) else v for h, v in zip(head, r)])
    for c in ws[1]:
        c.font = Font(bold=True, color="FFFFFF")
        c.fill = PatternFill("solid", fgColor="2F4858")
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions
    for i, h in enumerate(head, 1):
        width = max(len(h), *(len(str(r[i - 1])) for r in body)) + 2
        ws.column_dimensions[get_column_letter(i)].width = min(width, 70)
    if "Purpose" in head:  # long plain-language text: fixed width, wrapped
        col = head.index("Purpose") + 1
        ws.column_dimensions[get_column_letter(col)].width = 60
        for row in ws.iter_rows(min_row=2, min_col=col, max_col=col):
            row[0].alignment = Alignment(wrap_text=True, vertical="top")
    total = sum(float(r[head.index("Total_mass_g")] or 0) for r in body)
    ws.append([])
    tot = [""] * len(head)
    tot[0], tot[head.index("Total_mass_g")] = "TOTAL modelled mass (g)", round(total, 1)
    ws.append(tot)
    ws.cell(ws.max_row, 1).font = Font(bold=True)
    out = os.path.splitext(csv_path)[0] + ".xlsx"
    wb.save(out)
    return out


if __name__ == "__main__":
    print("wrote", write_xlsx())
