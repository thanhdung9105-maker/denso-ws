from pptx import Presentation

prs = Presentation('c:/Users/thanh/denso_ws/temp_presentation.pptx')

keywords = ['415', '350', '35 ppm', '35ppm', '15 mẫu', '15 mau', '700', '1400', '1000']

for idx, slide in enumerate(prs.slides):
    s_num = idx + 1
    for sh in slide.shapes:
        text = ""
        if sh.has_text_frame:
            text = " ".join([p.text for p in sh.text_frame.paragraphs])
        elif sh.has_table:
            text = " ".join([c.text for row in sh.table.rows for c in row.cells])
        for kw in keywords:
            if kw in text:
                print(f"Slide {s_num} contains '{kw}': {text[:80]}...")

