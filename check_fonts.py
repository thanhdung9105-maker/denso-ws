from pptx import Presentation

prs = Presentation('c:/Users/thanh/denso_ws/temp_presentation.pptx')
fonts = set()
for s in prs.slides:
    for sh in s.shapes:
        if sh.has_text_frame:
            for p in sh.text_frame.paragraphs:
                for r in p.runs:
                    if r.font.name:
                        fonts.add(r.font.name)
print("Fonts used:", fonts)

