from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
s20 = prs.slides[19]
t_shape = [sh for sh in s20.shapes if sh.has_table][0]
t = t_shape.table

print("Table cols widths:", [c.width for c in t.columns])
for r_i in range(min(3, len(t.rows))):
    for c_i in range(len(t.columns)):
        cell = t.rows[r_i].cells[c_i]
        p = cell.text_frame.paragraphs[0]
        f_size = p.font.size.pt if p.font.size else "inherit"
        print(f"R{r_i}C{c_i}: '{cell.text}' | font={f_size}")
